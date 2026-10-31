// Contract test for the range and image sensor interfaces: a host-side ray caster and
// renderer for a flat wall, and ideal adapters (no errors) driven through the interfaces.
#include "hakoniwa/sensor_adapter/camera_sensor_adapter.hpp"
#include "hakoniwa/sensor_adapter/lidar_sensor_adapter.hpp"
#include "hakoniwa/sensor_adapter/rangefinder_sensor_adapter.hpp"

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <limits>
#include <memory>
#include <vector>

namespace sa = hakoniwa::sensor_adapter;

namespace {

int failures = 0;
constexpr double kPi = 3.14159265358979323846;
constexpr double kInf = std::numeric_limits<double>::infinity();

void check(bool condition, const char* what)
{
    if (!condition) {
        std::fprintf(stderr, "FAIL: %s\n", what);
        ++failures;
    }
}

bool near(double a, double b, double tol = 1e-9) { return std::fabs(a - b) <= tol; }

const std::vector<sa::LogColumn> kNoColumns;

// Host side: a wall facing the sensor at x = kWall (sensor axes), object id 7.
constexpr double kWall = 4.0;
class WallCaster : public sa::IRayCaster {
public:
    bool cast(const sa::Ray* rays, std::size_t count, double max_distance, sa::RayHit* hits, std::string&) override
    {
        for (std::size_t i = 0; i < count; ++i) {
            const auto& r = rays[i];
            hits[i] = {};
            if (r.direction.x <= 0.0) continue;
            const double d = (kWall - r.origin.x) / r.direction.x;
            if (d < 0.0 || d > max_distance) continue;
            hits[i] = {true, d, 7, {-1.0, 0.0, 0.0}};
        }
        ++calls;
        return true;
    }
    int calls{0};
};

// Host side: renders the same wall as planar depth.
class WallRenderer : public sa::IImageRenderer {
public:
    bool render(const sa::RenderRequest& request, sa::RenderResult& result, std::string&) override
    {
        depth_.assign(static_cast<std::size_t>(request.width) * request.height,
                      kWall > request.far_clip ? std::numeric_limits<float>::infinity() : static_cast<float>(kWall));
        rgb_.assign(static_cast<std::size_t>(request.width) * request.height * 3, 128);
        result.depth = request.depth ? depth_.data() : nullptr;
        result.rgb = request.rgb ? rgb_.data() : nullptr;
        return true;
    }

private:
    std::vector<float> depth_;
    std::vector<std::uint8_t> rgb_;
};

class Timing {
public:
    Timing(std::uint64_t step_ns, std::uint64_t period_ns) : step_ns_(step_ns), period_ns_(period_ns) {}
    int advance(std::uint64_t t, std::string& error)
    {
        if (t != next_) {
            error = "sim_time_ns must advance by physics_step_ns from 0";
            return -1;
        }
        next_ += step_ns_;
        return t % period_ns_ == 0 ? 1 : 0;
    }
    void reset() { next_ = 0; }
    std::uint64_t step_ns() const { return step_ns_; }
    std::uint64_t period_ns() const { return period_ns_; }

private:
    std::uint64_t step_ns_, period_ns_, next_{0};
};

// Ideal single-beam rangefinder, 10 Hz, 0.1-10 m.
class IdealRangefinder : public sa::IRangefinderSensorAdapter {
public:
    explicit IdealRangefinder(const sa::RangefinderAdapterSetup& s) : caster_(s.ray_caster), timing_(s.physics_step_ns, 100000000) {}
    bool step(const sa::SensorPoseTruth& truth, sa::RangeSample& out, std::string& error) override
    {
        const int m = timing_.advance(truth.sim_time_ns, error);
        if (m < 0) return false;
        if (m == 1) {
            sa::Ray ray{};
            sa::RayHit hit{};
            if (!caster_->cast(&ray, 1, 10.0, &hit, error)) return false;
            last_ = {true, truth.sim_time_ns, sa::RadiationType::kLaser, 0.0, 0.1, 10.0,
                     hit.hit ? (hit.distance < 0.1 ? -kInf : hit.distance) : kInf, 0.0, 0};
        } else {
            last_.updated = false;
        }
        out = last_;
        return true;
    }
    bool reset(std::string&) override { timing_.reset(); last_ = {}; return true; }
    const std::vector<sa::LogColumn>& log_columns() const override { return kNoColumns; }
    void log_values(std::vector<double>& v) const override { v.clear(); }
    sa::RangefinderAdapterCapabilities capabilities() const override
    {
        sa::RangefinderAdapterCapabilities c{};
        c.requires_ray_caster = true;
        c.deterministic_with_seed = true;
        return c;
    }

private:
    sa::IRayCaster* caster_;
    Timing timing_;
    sa::RangeSample last_{};
};

// Ideal rotating 2D LiDAR: 8 rays over one turn at 10 Hz, cast as the rotation reaches them.
class IdealRotatingLidar : public sa::ILidarSensorAdapter {
public:
    static constexpr std::uint32_t kCount = 8;
    explicit IdealRotatingLidar(const sa::LidarAdapterSetup& s) : caster_(s.ray_caster), timing_(s.physics_step_ns, 100000000)
    {
        ranges_.assign(kCount, 0.0F);
        intensities_.assign(kCount, 0.0F);
    }
    bool step(const sa::SensorPoseTruth& truth, sa::LidarScan& out, std::string& error) override
    {
        if (timing_.advance(truth.sim_time_ns, error) < 0) return false;
        const std::uint64_t in_scan = truth.sim_time_ns % timing_.period_ns();
        if (in_scan == 0) {
            start_ns_ = truth.sim_time_ns;
            next_ray_ = 0;
            points_.clear();
        }
        // Rays whose time falls in this step: ray i fires at i * period / kCount.
        while (next_ray_ < kCount && next_ray_ * (timing_.period_ns() / kCount) <= in_scan) {
            const double a = -kPi + next_ray_ * (2 * kPi / kCount);
            sa::Ray ray{{0, 0, 0}, {std::cos(a), std::sin(a), 0.0}};
            sa::RayHit hit{};
            if (!caster_->cast(&ray, 1, 10.0, &hit, error)) return false;
            ranges_[next_ray_] = hit.hit ? static_cast<float>(hit.distance) : std::numeric_limits<float>::infinity();
            if (hit.hit) {
                sa::LidarPoint p{};
                p.x = static_cast<float>(hit.distance * ray.direction.x);
                p.y = static_cast<float>(hit.distance * ray.direction.y);
                p.time_offset = static_cast<float>((truth.sim_time_ns - start_ns_) * 1e-9);
                p.object_id = hit.object_id;
                points_.push_back(p);
            }
            ++next_ray_;
        }
        sa::LidarScan scan{};
        if (next_ray_ == kCount && in_scan + timing_.step_ns() == timing_.period_ns()) {
            scan.updated = true;
            scan.sample_time_ns = start_ns_;
            scan.pose_at_end = truth;
            scan.angle_min = -kPi;
            scan.angle_increment = 2 * kPi / kCount;
            scan.angle_max = -kPi + (kCount - 1) * scan.angle_increment;
            scan.horizontal_count = kCount;
            scan.vertical_count = 1;
            scan.scan_time = timing_.period_ns() * 1e-9;
            scan.time_increment = scan.scan_time / kCount;
            scan.range_min = 0.1;
            scan.range_max = 10.0;
            scan.ranges = ranges_.data();
            scan.intensities = intensities_.data();
            scan.points = points_.data();
            scan.point_count = static_cast<std::uint32_t>(points_.size());
        }
        out = scan;
        return true;
    }
    bool reset(std::string&) override { timing_.reset(); next_ray_ = 0; points_.clear(); return true; }
    const std::vector<sa::LogColumn>& log_columns() const override { return kNoColumns; }
    void log_values(std::vector<double>& v) const override { v.clear(); }
    sa::LidarAdapterCapabilities capabilities() const override
    {
        sa::LidarAdapterCapabilities c{};
        c.requires_ray_caster = true;
        c.rotating_scan = true;
        c.object_id = true;
        c.deterministic_with_seed = true;
        return c;
    }

private:
    sa::IRayCaster* caster_;
    Timing timing_;
    std::uint32_t next_ray_{0};
    std::uint64_t start_ns_{0};
    std::vector<float> ranges_, intensities_;
    std::vector<sa::LidarPoint> points_;
};

// Ideal depth camera from the renderer, 4x3, 90 degrees, perspective converted from planar.
class IdealDepthCamera : public sa::IDepthCameraSensorAdapter {
public:
    explicit IdealDepthCamera(const sa::DepthCameraAdapterSetup& s) : renderer_(s.renderer), timing_(s.physics_step_ns, 50000000) {}
    bool step(const sa::SensorPoseTruth& truth, sa::DepthImage& out, std::string& error) override
    {
        const int m = timing_.advance(truth.sim_time_ns, error);
        if (m < 0) return false;
        if (m == 1) {
            sa::RenderRequest req{4, 3, kPi / 2, 0.1, 10.0, false, true};
            sa::RenderResult res{};
            if (!renderer_->render(req, res, error)) return false;
            const double f = 2.0 / std::tan(req.horizontal_fov / 2);  // width / 2 / tan(fov / 2)
            data_.resize(12);
            for (std::uint32_t v = 0; v < 3; ++v)
                for (std::uint32_t u = 0; u < 4; ++u) {
                    const double x = (u + 0.5 - 2.0) / f, y = (v + 0.5 - 1.5) / f;
                    data_[v * 4 + u] = static_cast<float>(res.depth[v * 4 + u] * std::sqrt(1 + x * x + y * y));
                }
            last_ = {};
            last_.updated = true;
            last_.sample_time_ns = truth.sim_time_ns;
            last_.depth_type = sa::DepthType::kPerspective;
            last_.width = 4;
            last_.height = 3;
            last_.data = data_.data();
            last_.info.width = 4;
            last_.info.height = 3;
            last_.info.k = {f, 0, 2.0, 0, f, 1.5, 0, 0, 1};
        } else {
            last_.updated = false;
        }
        out = last_;
        return true;
    }
    bool reset(std::string&) override { timing_.reset(); last_ = {}; return true; }
    const std::vector<sa::LogColumn>& log_columns() const override { return kNoColumns; }
    void log_values(std::vector<double>& v) const override { v.clear(); }
    sa::DepthCameraAdapterCapabilities capabilities() const override
    {
        sa::DepthCameraAdapterCapabilities c{};
        c.requires_renderer = true;
        c.perspective = true;
        return c;
    }

private:
    sa::IImageRenderer* renderer_;
    Timing timing_;
    std::vector<float> data_;
    sa::DepthImage last_{};
};

sa::SensorPoseTruth at(std::uint64_t k) { return {k * 1000000, {0, 0, 10}, {}}; }

void rangefinder()
{
    WallCaster caster;
    sa::RangefinderAdapterSetup s{};
    s.ray_caster = &caster;
    check(s.sensor_name == "rangefinder", "rangefinder default name");
    std::unique_ptr<sa::IRangefinderSensorAdapter> a = std::make_unique<IdealRangefinder>(s);
    check(a->capabilities().requires_ray_caster && !a->capabilities().requires_renderer, "rangefinder declares the ray caster");
    int updates = 0;
    std::string e;
    for (std::uint64_t k = 0; k < 1000; ++k) {
        sa::RangeSample r{};
        check(a->step(at(k), r, e), "rangefinder step");
        if (r.updated) {
            ++updates;
            check(near(r.range, kWall), "range to the wall");
        }
    }
    check(updates == 10 && caster.calls == 10, "10 Hz over 1 s, one cast each");
    sa::RangeSample r{};
    check(!a->step(at(5), r, e) && !e.empty(), "a time jump is an error");
}

void lidar()
{
    WallCaster caster;
    sa::LidarAdapterSetup s{};
    s.ray_caster = &caster;
    std::unique_ptr<sa::ILidarSensorAdapter> a = std::make_unique<IdealRotatingLidar>(s);
    std::string e;
    int scans = 0;
    for (std::uint64_t k = 0; k < 300; ++k) {
        sa::LidarScan scan{};
        check(a->step(at(k), scan, e), "lidar step");
        if (!scan.updated) continue;
        ++scans;
        check(scan.horizontal_count == 8 && scan.vertical_count == 1, "2D scan layout");
        check(scan.sample_time_ns == (k - 99) * 1000000, "scan time is its first ray");
        // Rays at 0 and +-45 degrees hit the wall; the others are +inf (REP 117).
        int hits = 0;
        for (std::uint32_t i = 0; i < 8; ++i) {
            const double angle = scan.angle_min + i * scan.angle_increment;
            if (std::isfinite(scan.ranges[i])) {
                ++hits;
                check(near(scan.ranges[i], kWall / std::cos(angle), 1e-5), "range along the ray");
            } else {
                check(scan.ranges[i] > 0, "no return is +inf");
            }
        }
        check(hits == 3 && scan.point_count == 3, "three rays hit the wall");
        bool rising = true;
        for (std::uint32_t i = 1; i < scan.point_count; ++i)
            rising &= scan.points[i].time_offset > scan.points[i - 1].time_offset;
        check(rising, "points carry their own time in the scan");
        check(scan.points[0].object_id == 7 && near(scan.points[1].x, kWall, 1e-5), "points in sensor axes with object id");
    }
    check(scans == 3, "three complete scans in 0.3 s");
}

void depth_camera()
{
    WallRenderer renderer;
    sa::DepthCameraAdapterSetup s{};
    s.renderer = &renderer;
    std::unique_ptr<sa::IDepthCameraSensorAdapter> a = std::make_unique<IdealDepthCamera>(s);
    std::string e;
    sa::DepthImage img{};
    check(a->step(at(0), img, e) && img.updated, "depth frame at time 0");
    check(img.depth_type == sa::DepthType::kPerspective && img.width == 4 && img.height == 3, "depth layout");
    check(img.data[5] > kWall && img.data[0] > img.data[5], "perspective depth grows away from the centre");
    // The fixed optical frame: forward -> +z, left -> -x, up -> -y.
    const sa::Quaternion q = img.info.sensor_to_optical;
    auto rotate = [&q](sa::Vector3 v) {
        const double w = q.w, x = q.x, y = q.y, z = q.z;
        return sa::Vector3{(1 - 2 * (y * y + z * z)) * v.x + 2 * (x * y - z * w) * v.y + 2 * (x * z + y * w) * v.z,
                           2 * (x * y + z * w) * v.x + (1 - 2 * (x * x + z * z)) * v.y + 2 * (y * z - x * w) * v.z,
                           2 * (x * z - y * w) * v.x + 2 * (y * z + x * w) * v.y + (1 - 2 * (x * x + y * y)) * v.z};
    };
    const auto f = rotate({1, 0, 0}), l = rotate({0, 1, 0}), u = rotate({0, 0, 1});
    check(near(f.z, 1) && near(l.x, -1) && near(u.y, -1), "sensor_to_optical maps FLU to the optical frame");
}

}  // namespace

int main()
{
    rangefinder();
    lidar();
    depth_camera();
    if (failures == 0) std::printf("range_image_adapter_contract: OK\n");
    return failures == 0 ? EXIT_SUCCESS : EXIT_FAILURE;
}
