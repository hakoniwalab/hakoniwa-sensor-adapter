// The reference implementation, driven through the interfaces: the LiDAR against a
// host-side wall, the force-torque and joint sensors with given truths, and the
// descriptor. It returns the true values only, so most checks compare with the truth.
// Each check names the specification rule it covers.
#include "hakoniwa/sensor_adapter/reference/reference_implementation.hpp"

#include <cmath>
#include <cstdio>
#include <fstream>
#include <limits>
#include <string>
#include <vector>

namespace sa = hakoniwa::sensor_adapter;

namespace {

int failures = 0;
constexpr std::uint64_t kStepNs = 2000000;  // 2 ms

void check(bool condition, const std::string& what)
{
    if (!condition) {
        std::fprintf(stderr, "FAIL: %s\n", what.c_str());
        ++failures;
    }
}

bool near(double a, double b, double tol = 1e-5) { return std::fabs(a - b) <= tol; }

std::string write_file(const std::string& name, const std::string& text)
{
    const std::string path = std::string(REFERENCE_TEST_DIR) + "/" + name;
    std::ofstream(path) << text;
    return path;
}

// A wall at x = wall (sensor axes), object id 7. Rays that do not point forward miss.
class WallCaster : public sa::IRayCaster {
public:
    explicit WallCaster(double wall) : wall_(wall) {}
    bool cast(const sa::Ray* rays, std::size_t count, double max_distance, sa::RayHit* hits, std::string& error) override
    {
        if (fail) {
            error = "host failure";
            return false;
        }
        last_max_distance = max_distance;
        for (std::size_t i = 0; i < count; ++i) {
            hits[i] = {};
            if (rays[i].direction.x <= 1e-12) continue;
            const double d = (wall_ - rays[i].origin.x) / rays[i].direction.x;
            if (d < 0.0 || d > max_distance) continue;
            hits[i] = {true, d, 7, {-1.0, 0.0, 0.0}};
        }
        ++calls;
        return true;
    }
    double wall_;
    bool fail{false};
    int calls{0};
    double last_max_distance{0.0};
};

sa::SensorPoseTruth pose(std::uint64_t step)
{
    sa::SensorPoseTruth t;
    t.sim_time_ns = step * kStepNs;
    return t;
}

std::unique_ptr<sa::ILidarSensorAdapter> make_lidar(const std::string& json, sa::IRayCaster* caster, std::string& error)
{
    sa::LidarAdapterSetup setup;
    setup.config_path = write_file("lidar.json", json);
    setup.physics_step_ns = kStepNs;
    setup.ray_caster = caster;
    return sa::reference::implementation().create_lidar(setup, error);
}

// 5 x 3 grid, +-0.4 rad horizontally, +-0.2 rad vertically, 0.1-10 m, 125 Hz (4 steps).
const std::string kLidar = R"({"type": "lidar", "update_rate": 125, "scan_mode": "instant",
    "scan": {"horizontal": {"samples": 5, "min_angle": -0.4, "max_angle": 0.4},
             "vertical": {"samples": 3, "min_angle": -0.2, "max_angle": 0.2}},
    "range": {"min": 0.1, "max": 10.0}})";

void test_lidar_truth()
{
    WallCaster wall(4.0);
    std::string error;
    auto lidar = make_lidar(kLidar, &wall, error);
    check(lidar != nullptr, "lidar: created (" + error + ")");
    if (!lidar) return;
    const auto caps = lidar->capabilities();
    check(caps.requires_ray_caster && !caps.noise && !caps.resolution && !caps.rotating_scan && !caps.intensity &&
              !caps.range_dependent_accuracy,
          "capabilities: only the ray caster, no error of any kind");
    sa::LidarScan out;
    for (std::uint64_t k = 0; k < 9; ++k) {
        check(lidar->step(pose(k), out, error), "lidar step " + std::to_string(k));
        check(out.updated == ((k % 4) == 0), "a scan on the first step of each period (RIS-LID-070, RIS-COM-070), k=" +
                                                 std::to_string(k));
    }
    check(wall.calls == 3, "one cast per scan");
    check(out.horizontal_count == 5 && out.vertical_count == 3, "grid size (RIS-LID-010)");
    check(near(out.angle_increment, 0.2) && near(out.vertical_angle_increment, 0.2), "increments (RIS-LID-010)");
    check(out.scan_time == 0.0 && out.time_increment == 0.0, "instant: scan_time 0 (RIS-LID-070)");
    check(out.sample_time_ns == 8 * kStepNs, "sample time of the last scan");
    // Row 0 is the lowest (el -0.2), column 0 the first angle (az -0.4) (RIS-LID-020, -030).
    check(near(out.ranges[0], 4.0 / (std::cos(-0.2) * std::cos(-0.4)), 1e-4), "true distance of row 0, column 0");
    check(near(out.ranges[1 * 5 + 2], 4.0), "true distance of the centre ray");
    check(out.point_count == 15, "every finite ray is a point (RIS-LID-090)");
    check(near(out.points[0].x, 4.0, 1e-4) && out.points[0].ring == 0 && out.points[0].object_id == 7 &&
              out.points[0].time_offset == 0.0F,
          "point position, ring, object id, time_offset 0 (RIS-LID-090)");
    check(out.intensities[0] == 0.0F && out.flags == 0, "no intensity, no flag (RIS-LID-100)");
    check(near(wall.last_max_distance, 10.0), "cast length is the maximum (RIS-LID-120)");
    std::vector<double> values;
    lidar->log_values(values);
    check(lidar->log_columns().size() == 1 && lidar->log_columns()[0].name == "points" && values[0] == 15.0,
          "log column points (RIS-LID-130)");
    // Between scans the last scan is held (RIS-LID-110, RIS-COM-080).
    check(lidar->step(pose(9), out, error) && !out.updated && out.point_count == 15, "held between scans");
}

void test_lidar_out_of_range()
{
    std::string error;
    sa::LidarScan out;
    // Column 0 forward hits a wall nearer than the minimum; column 1 backwards hits nothing.
    WallCaster near_wall(0.05);
    auto lidar = make_lidar(R"({"type": "lidar", "update_rate": 500, "scan_mode": "instant",
        "scan": {"horizontal": {"samples": 2, "min_angle": 0.0, "max_angle": 3.14159265358979}},
        "range": {"min": 0.1, "max": 10.0}, "no_return": "inf"})",
                            &near_wall, error);
    check(lidar && lidar->step(pose(0), out, error), "out of range: step (" + error + ")");
    if (!lidar) return;
    check(out.vertical_count == 1 && out.vertical_angle_min == 0.0, "no vertical axis: one row at 0 (RIS-LID-010)");
    check(std::isinf(out.ranges[0]) && out.ranges[0] < 0, "nearer than the minimum: -inf (RIS-COM-120, REP 117)");
    check(std::isinf(out.ranges[1]) && out.ranges[1] > 0, "nothing hit: +inf (RIS-COM-120, REP 117)");
    check(out.point_count == 0, "infinite values are not points (RIS-LID-090)");
}

void test_lidar_refusals()
{
    WallCaster wall(4.0);
    std::string error;
    auto refused = [&](const std::string& json, const std::string& word, const std::string& what) {
        const bool ok = make_lidar(json, &wall, error) == nullptr && error.find(word) != std::string::npos;
        check(ok, what + " (error: " + error + ")");
    };
    const std::string grid = R"("scan": {"horizontal": {"samples": 3, "min_angle": -0.1, "max_angle": 0.1}})";
    refused(R"({"type": "lidar", "update_rate": 10, "scan_mode": "instant", )" + grid +
                R"(, "range": {"min": 0.1, "max": 10.0}, "accuracy": [{"range": {"min": 0, "max": 10}, "stddev": 0.01}]})",
            "accuracy", "noise (accuracy) is refused, naming the key");
    refused(R"({"type": "lidar", "update_rate": 10, "scan_mode": "instant", )" + grid +
                R"(, "range": {"min": 0.1, "max": 10.0, "resolution": 0.01}})",
            "resolution", "resolution is refused, naming the key");
    refused(R"({"type": "lidar", "update_rate": 10, "scan_mode": "rotating", )" + grid +
                R"(, "range": {"min": 0.1, "max": 10.0}})",
            "instant", "a rotating scan is refused");
    refused(R"({"type": "lidar", "update_rate": 10, )" + grid + R"(, "range": {"min": 0.1, "max": 10.0}})", "instant",
            "scan_mode left at its default (rotating) is refused");
    refused(R"({"type": "lidar", "update_rate": 10, "scan_mode": "instant", )" + grid +
                R"(, "range": {"min": 0.1, "max": 10.0}, "no_return": "limit"})",
            "no_return", "no_return limit is refused");
    refused(R"({"type": "imu", "update_rate": 10})", "type", "a wrong type is refused (RIS-COM-180)");
    refused(R"({"type": "lidar", "update_rate": 10, "scan_mode": "instant", )" + grid +
                R"(, "range": {"min": 2.0, "max": 1.0}})",
            "min", "min >= max is refused (RIS-COM-170)");
    check(make_lidar(kLidar, nullptr, error) == nullptr, "no ray caster is refused (RIS-COM-040)");
}

void test_lidar_errors_and_reset()
{
    WallCaster wall(4.0);
    std::string error;
    auto lidar = make_lidar(kLidar, &wall, error);
    sa::LidarScan out;
    check(lidar && lidar->step(pose(0), out, error), "step 0");
    if (!lidar) return;
    check(!lidar->step(pose(2), out, error), "a skipped time is an error (RIS-COM-050)");
    check(lidar->step(pose(1), out, error), "and the state did not advance");
    for (std::uint64_t k = 2; k < 4; ++k) check(lidar->step(pose(k), out, error), "steps 2-3");
    wall.fail = true;
    check(!lidar->step(pose(4), out, error) && error.find("host failure") != std::string::npos,
          "a ray cast failure is reported");
    wall.fail = false;
    check(lidar->step(pose(4), out, error) && out.updated, "and the step can be retried");
    check(lidar->reset(error) && lidar->step(pose(0), out, error) && out.updated,
          "reset: the next step starts at 0 (RIS-COM-110)");
}

template <class Make>
void refused_definitions(Make make, const std::string& type, const std::string& key_json, const std::string& key)
{
    std::string error;
    const bool ok =
        make(R"({"type": ")" + type + R"(", "update_rate": 100, )" + key_json + "}", error) == nullptr &&
        error.find(key) != std::string::npos;
    check(ok, type + ": \"" + key + "\" is refused, naming the key (error: " + error + ")");
}

std::unique_ptr<sa::IForceTorqueSensorAdapter> make_ft(const std::string& json, std::string& error)
{
    sa::ForceTorqueAdapterSetup setup;
    setup.config_path = write_file("ft.json", json);
    setup.physics_step_ns = kStepNs;
    return sa::reference::implementation().create_force_torque(setup, error);
}

sa::ForceTorqueTruth ft_truth(std::uint64_t step, double fz, double tx)
{
    sa::ForceTorqueTruth t;
    t.sim_time_ns = step * kStepNs;
    t.wrench.force.z = fz;
    t.wrench.torque.x = tx;
    return t;
}

void test_force_torque()
{
    std::string error;
    auto ft = make_ft(R"({"type": "force_torque", "update_rate": 250})", error);  // 4 ms = 2 steps
    check(ft != nullptr, "ft: created (" + error + ")");
    if (!ft) return;
    const auto caps = ft->capabilities();
    check(!caps.range_clamp && !caps.tare && !caps.noise && !caps.delay && !caps.bias && !caps.overload,
          "ft capabilities: no error of any kind");
    sa::ForceTorqueSample out;
    check(ft->step(ft_truth(0, -3.0, 0.5), out, error) && out.updated && near(out.wrench.force.z, -3.0) &&
              near(out.wrench.torque.x, 0.5),
          "the truth on the sampling step (FTS-ADP-300)");
    check(ft->step(ft_truth(1, -5.0, 0.0), out, error) && !out.updated && near(out.wrench.force.z, -3.0),
          "held between samples (FTS-ADP-030)");
    check(ft->step(ft_truth(2, 1.0e6, 0.0), out, error) && out.updated && near(out.wrench.force.z, 1.0e6) && out.flags == 0,
          "a large truth is returned as is (no range)");
    check(!ft->tare(error), "no tare");
    std::vector<double> values;
    ft->log_values(values);
    check(ft->log_columns().size() == 18 && values.size() == 18 && ft->log_columns()[0].name == "true_fx",
          "log columns (FTS-ADP-610)");
    check(!ft->step(ft_truth(9, 0.0, 0.0), out, error), "a skipped time is an error (FTS-ADP-020)");
    check(ft->reset(error) && ft->step(ft_truth(0, 1.0, 0.0), out, error) && out.updated, "reset (FTS-ADP-050)");
    refused_definitions(make_ft, "force_torque", R"("range": {"fxy": 200, "fz": 200, "txy": 10, "tz": 6.5})", "range");
    refused_definitions(make_ft, "force_torque", R"("noise": {"fxy": 0.1})", "noise");
}

std::unique_ptr<sa::IJointSensorAdapter> make_joint(const std::string& json, std::string& error)
{
    sa::JointSensorAdapterSetup setup;
    setup.config_path = write_file("joint.json", json);
    setup.physics_step_ns = kStepNs;
    return sa::reference::implementation().create_joint(setup, error);
}

void test_joint()
{
    std::string error;
    auto joint = make_joint(R"({"type": "joint", "update_rate": 500})", error);  // every step
    check(joint != nullptr, "joint: created (" + error + ")");
    if (!joint) return;
    const auto caps = joint->capabilities();
    check(!caps.encoder_counts && !caps.noise && !caps.delay && !caps.backlash && !caps.offset && caps.torque_sensor,
          "joint capabilities: true values, the joint torque as the effort");
    sa::JointSensorTruth truth;
    truth.position = 0.3;
    truth.velocity = -1.2;
    truth.actuator_torque = 2.0;
    truth.transmitted_torque = 1.5;
    sa::JointSensorSample out;
    check(joint->step(truth, out, error) && out.updated && near(out.position, 0.3) && near(out.velocity, -1.2) &&
              out.has_effort && near(out.effort, 1.5) && out.encoder_count == 0,
          "true position, velocity and transmitted torque (JNT-ADP-290, -310, -410)");
    truth.sim_time_ns = kStepNs;
    truth.has_transmitted_torque = false;
    check(joint->step(truth, out, error) && near(out.effort, 2.0), "actuator torque when there is no transmitted torque");
    std::vector<double> values;
    joint->log_values(values);
    check(joint->log_columns().size() == 5 && values.size() == 5 && joint->log_columns()[0].name == "true_position",
          "log columns (JNT-ADP-610)");
    refused_definitions(make_joint, "joint", R"("encoder": {"counts_per_rev": 4096})", "encoder");
    refused_definitions(make_joint, "joint", R"("delay": 0.001)", "delay");
}

void test_descriptor()
{
    const auto* entry = HAKO_SENSOR_ADAPTER_IMPLEMENTATION_ENTRY(reference)();
    check(entry != nullptr && std::string(entry->name) == "reference", "entry point and name");
    const auto types = sa::provided_sensor_types(*entry);
    check(types == std::vector<std::string>{"lidar", "force_torque", "joint"}, "provides lidar, force_torque and joint");
    check(entry->create_imu == nullptr && entry->create_rangefinder == nullptr && entry->create_depth_camera == nullptr,
          "other types are not provided");
}

}  // namespace

int main()
{
    test_lidar_truth();
    test_lidar_out_of_range();
    test_lidar_refusals();
    test_lidar_errors_and_reset();
    test_force_torque();
    test_joint();
    test_descriptor();
    if (failures != 0) {
        std::fprintf(stderr, "%d check(s) failed\n", failures);
        return 1;
    }
    std::printf("reference implementation: all checks passed\n");
    return 0;
}
