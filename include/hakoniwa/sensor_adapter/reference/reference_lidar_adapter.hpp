#pragma once

// Reference LiDAR adapter: returns the true distances, nothing else.
//
// It follows the LiDAR specification (docs/spec/lidar.md) for the scan grid
// (RIS-LID-010 to -030), the instant scan (RIS-LID-070, -080), the points
// (RIS-LID-090 with time_offset 0) and the common time rules (RIS-COM-050 to -080),
// and REP 117 for values out of range (RIS-COM-120, no_return "inf").
//
// It adds no error of any kind: no rotating scan, no noise (accuracy), no
// resolution, no clamping. A definition that asks for one of them is refused,
// naming the key. Those belong to implementations that model a real sensor.
//
// It measures the world only through the host's IRayCaster.

#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "hakoniwa/sensor_adapter/lidar_sensor_adapter.hpp"
#include "hakoniwa/sensor_adapter/reference/reference_definition.hpp"

namespace hakoniwa::sensor_adapter::reference {

class ReferenceLidarAdapter final : public ILidarSensorAdapter {
public:
    /// Creates the adapter from setup.config_path. Returns nullptr and sets error on failure.
    static std::unique_ptr<ILidarSensorAdapter> create(const LidarAdapterSetup& setup, std::string& error)
    {
        auto adapter = std::unique_ptr<ReferenceLidarAdapter>(new ReferenceLidarAdapter());
        if (!adapter->configure(setup, error)) {
            return nullptr;
        }
        return adapter;
    }

    bool step(const SensorPoseTruth& truth, LidarScan& out, std::string& error) override
    {
        const std::uint64_t expected = step_index_ * physics_step_ns_;
        if (truth.sim_time_ns != expected) {
            error = sensor_name_ + ": step time " + std::to_string(truth.sim_time_ns) + " ns, expected " +
                    std::to_string(expected) + " ns";
            return false;
        }
        const bool sample_now = (step_index_ % period_) == 0;
        if (sample_now && !scan(truth, error)) {
            return false;
        }
        out = scan_;
        out.updated = sample_now;
        out.ranges = ranges_.data();
        out.intensities = intensities_.data();
        out.points = points_.empty() ? nullptr : points_.data();
        out.point_count = static_cast<std::uint32_t>(points_.size());
        ++step_index_;
        return true;
    }

    LidarAdapterCapabilities capabilities() const override
    {
        LidarAdapterCapabilities c;
        c.requires_ray_caster = true;
        c.object_id = true;
        c.deterministic_with_seed = true;  // no random values at all
        c.thread_safe_across_instances = true;
        return c;
    }

    bool reset(std::string& error) override
    {
        (void)error;
        step_index_ = 0;
        clear_output();
        return true;
    }

    const std::vector<LogColumn>& log_columns() const override { return columns_; }

    void log_values(std::vector<double>& values) const override
    {
        values.assign(1, static_cast<double>(points_.size()));
    }

private:
    ReferenceLidarAdapter() = default;

    struct Axis {
        std::uint32_t samples{1};
        double min_angle{0.0};
        double max_angle{0.0};
        double angle(std::uint32_t i) const
        {
            if (samples == 1) {
                return min_angle;
            }
            return min_angle + (max_angle - min_angle) * static_cast<double>(i) / static_cast<double>(samples - 1);
        }
        double increment() const
        {
            return samples == 1 ? 0.0 : (max_angle - min_angle) / static_cast<double>(samples - 1);
        }
    };

    static bool read_axis(const detail::Json& scan, const char* name, Axis& axis, std::string& error)
    {
        const std::string where = std::string("scan.") + name;
        const auto& a = scan.at(name);
        if (!a.is_object()) {
            error = where + " must be an object";
            return false;
        }
        for (const auto& item : a.items()) {
            if (item.key() != "samples" && item.key() != "min_angle" && item.key() != "max_angle") {
                error = where + ": unknown key \"" + item.key() + "\"";
                return false;
            }
        }
        if (!a.contains("samples") || !a.at("samples").is_number_integer() || a.at("samples").get<long long>() < 1 ||
            a.at("samples").get<long long>() > 1000000) {
            error = where + ".samples must be an integer from 1";
            return false;
        }
        axis.samples = static_cast<std::uint32_t>(a.at("samples").get<long long>());
        if (!detail::read_number(a, "min_angle", where, axis.min_angle, error) ||
            !detail::read_number(a, "max_angle", where, axis.max_angle, error)) {
            return false;
        }
        if (axis.max_angle < axis.min_angle) {
            error = where + ": max_angle is less than min_angle";
            return false;
        }
        return true;
    }

    bool configure(const LidarAdapterSetup& setup, std::string& error)
    {
        sensor_name_ = setup.sensor_name;
        physics_step_ns_ = setup.physics_step_ns;
        caster_ = setup.ray_caster;
        if (caster_ == nullptr) {
            error = sensor_name_ + ": the reference LiDAR needs the host's ray caster";
            return false;
        }
        detail::Json def;
        const std::string& path = setup.config_path;
        if (!detail::read_definition(path, def, error) ||
            !detail::check_keys(def, "lidar", {"update_rate", "scan_mode", "scan", "range", "no_return", "output"},
                                path, error)) {
            return false;
        }
        double update_rate = 0.0;
        if (!detail::read_number(def, "update_rate", path, update_rate, error) ||
            !detail::sample_period_steps(update_rate, physics_step_ns_, sensor_name_, period_, error)) {
            return false;
        }
        // The specification's default scan_mode is rotating, so the reference
        // implementation needs "instant" written explicitly.
        if (!def.contains("scan_mode") || def.at("scan_mode") != "instant") {
            error = path + ": the reference implementation returns the true distances of an instant scan; "
                           "write \"scan_mode\": \"instant\" (a rotating scan needs another implementation)";
            return false;
        }
        if (def.contains("no_return") && def.at("no_return") != "inf") {
            error = path + ": the reference implementation provides only \"no_return\": \"inf\"";
            return false;
        }
        if (!def.contains("scan") || !def.at("scan").is_object() || !def.at("scan").contains("horizontal")) {
            error = path + ": scan.horizontal is required";
            return false;
        }
        const auto& scan = def.at("scan");
        for (const auto& item : scan.items()) {
            if (item.key() != "horizontal" && item.key() != "vertical") {
                error = path + ": unknown key scan." + item.key();
                return false;
            }
        }
        if (!read_axis(scan, "horizontal", horizontal_, error)) {
            return false;
        }
        vertical_ = Axis{};
        if (scan.contains("vertical") && !read_axis(scan, "vertical", vertical_, error)) {
            return false;
        }
        if (!def.contains("range") || !def.at("range").is_object()) {
            error = path + ": range is required";
            return false;
        }
        const auto& range = def.at("range");
        for (const auto& item : range.items()) {
            if (item.key() != "min" && item.key() != "max") {
                error = path + ": the reference implementation does not provide range." + item.key();
                return false;
            }
        }
        if (!detail::read_number(range, "min", path + " range", range_min_, error) ||
            !detail::read_number(range, "max", path + " range", range_max_, error)) {
            return false;
        }
        if (!(range_min_ >= 0.0 && range_min_ < range_max_)) {
            error = path + ": range must satisfy 0 <= min < max";
            return false;
        }

        // Ray directions (RIS-LID-020), row-major: row = vertical sample (lowest first).
        const std::size_t h = horizontal_.samples;
        const std::size_t v = vertical_.samples;
        rays_.resize(h * v);
        for (std::size_t row = 0; row < v; ++row) {
            const double el = vertical_.angle(static_cast<std::uint32_t>(row));
            for (std::size_t col = 0; col < h; ++col) {
                const double az = horizontal_.angle(static_cast<std::uint32_t>(col));
                rays_[row * h + col].direction = {std::cos(el) * std::cos(az), std::cos(el) * std::sin(az), std::sin(el)};
            }
        }
        hits_.resize(h * v);
        columns_ = {{"points", ""}};
        clear_output();
        return true;
    }

    void clear_output()
    {
        ranges_.assign(rays_.size(), std::numeric_limits<float>::infinity());
        intensities_.assign(rays_.size(), 0.0F);
        points_.clear();
        scan_ = LidarScan{};
        scan_.angle_min = horizontal_.min_angle;
        scan_.angle_max = horizontal_.max_angle;
        scan_.angle_increment = horizontal_.increment();
        scan_.vertical_angle_min = vertical_.min_angle;
        scan_.vertical_angle_max = vertical_.max_angle;
        scan_.vertical_angle_increment = vertical_.increment();
        scan_.horizontal_count = horizontal_.samples;
        scan_.vertical_count = vertical_.samples;
        scan_.range_min = range_min_;
        scan_.range_max = range_max_;
    }

    // One instant scan: every ray at once, the true distance of each (RIS-LID-070, -090).
    bool scan(const SensorPoseTruth& truth, std::string& error)
    {
        if (!caster_->cast(rays_.data(), rays_.size(), range_max_, hits_.data(), error)) {
            error = sensor_name_ + ": ray cast failed: " + error;
            return false;
        }
        const double inf = std::numeric_limits<double>::infinity();
        const std::size_t h = horizontal_.samples;
        points_.clear();
        for (std::size_t i = 0; i < rays_.size(); ++i) {
            const RayHit& hit = hits_[i];
            double value = inf;  // nothing within the maximum (REP 117)
            if (hit.hit && hit.distance <= range_max_) {
                value = hit.distance < range_min_ ? -inf : hit.distance;
            }
            ranges_[i] = static_cast<float>(value);
            if (!std::isfinite(value)) {
                continue;
            }
            const Vector3& d = rays_[i].direction;
            LidarPoint p;
            p.x = static_cast<float>(value * d.x);
            p.y = static_cast<float>(value * d.y);
            p.z = static_cast<float>(value * d.z);
            p.ring = static_cast<std::uint16_t>(i / h);
            p.object_id = hit.object_id;
            points_.push_back(p);
        }
        scan_.sample_time_ns = truth.sim_time_ns;
        scan_.pose_at_end = truth;
        return true;
    }

    std::string sensor_name_;
    std::uint64_t physics_step_ns_{1000000};
    IRayCaster* caster_{nullptr};
    std::uint64_t period_{1};
    Axis horizontal_;
    Axis vertical_;
    double range_min_{0.0};
    double range_max_{0.0};

    std::vector<Ray> rays_;
    std::vector<RayHit> hits_;
    std::uint64_t step_index_{0};
    std::vector<float> ranges_;
    std::vector<float> intensities_;
    std::vector<LidarPoint> points_;
    LidarScan scan_{};
    std::vector<LogColumn> columns_;
};

}  // namespace hakoniwa::sensor_adapter::reference
