// Axis 0 of docs/evaluation/drone/evaluation-axes.md: the same truth into the Hakoniwa adapters and
// the Gazebo adapters with every error off, compared value by value.
//
// Usage: compare_axis0_no_noise OUT_DIR
// Writes one CSV per comparison into OUT_DIR and prints a summary.
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <functional>
#include <string>
#include <vector>

#include <gz/math/Angle.hh>
#include <gz/math/CoordinateVector3.hh>
#include <gz/math/SphericalCoordinates.hh>

#include "hakoniwa/sensor_adapter/gazebo/gz_baro_sensor_adapter.hpp"
#include "hakoniwa/sensor_adapter/gazebo/gz_gps_sensor_adapter.hpp"
#include "hakoniwa/sensor_adapter/gazebo/gz_imu_sensor_adapter.hpp"
#include "hakoniwa/sensor_adapter/gazebo/gz_mag_sensor_adapter.hpp"
#ifdef HAKO_SENSORS_WITH_AIRSIM
#include "hakoniwa/sensor_adapter/airsim/projectairsim_native_baro_adapter.hpp"
#include "hakoniwa/sensor_adapter/airsim/projectairsim_native_gps_adapter.hpp"
#include "hakoniwa/sensor_adapter/airsim/projectairsim_native_imu_adapter.hpp"
#include "hakoniwa/sensor_adapter/airsim/projectairsim_native_mag_adapter.hpp"
#endif
#include "hakoniwa_entry.hpp"

namespace sa = hakoniwa::sensor_adapter;
namespace gz_adapter = hakoniwa::sensor_adapter::gazebo;
#ifdef HAKO_SENSORS_WITH_AIRSIM
// Project AirSim's own core_sim sensors (the native adapters of hakoniwa-sensor-adapter-airsim).
namespace as_adapter = hakoniwa::sensor_adapter::airsim;
#endif

namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kDegToRad = kPi / 180.0;
constexpr std::uint64_t kStepNs = 1000000;  // 1 ms

// Simulation location (the Drone PRO default).
constexpr double kLat0 = 47.641468, kLon0 = -122.140165, kAlt0 = 121.321;

std::string config(const char* file)
{
    return std::string(HAKO_SENSORS_CONFIG_DIR) + "/" + file;
}

template <typename Setup>
Setup setup_for(const char* file, std::uint64_t step_ns = kStepNs)
{
    Setup s{};
    s.config_path = config(file);
    s.vehicle_name = "Drone";
    s.physics_step_ns = step_ns;
    return s;
}

[[noreturn]] void fail(const std::string& what)
{
    std::fprintf(stderr, "error: %s\n", what.c_str());
    std::exit(1);
}

template <typename T>
T require(T adapter, const std::string& error)
{
    if (!adapter) fail(error);
    return adapter;
}

sa::Quaternion yaw_pitch_roll_world_to_sensor(double yaw, double pitch, double roll)
{
    // Sensor-to-world rotation q = Rz(yaw) Ry(pitch) Rx(roll) in NWU/FLU; world-to-sensor is its conjugate.
    const double cy = std::cos(yaw / 2), sy = std::sin(yaw / 2);
    const double cp = std::cos(pitch / 2), sp = std::sin(pitch / 2);
    const double cr = std::cos(roll / 2), sr = std::sin(roll / 2);
    const double w = cy * cp * cr + sy * sp * sr, x = cy * cp * sr - sy * sp * cr;
    const double y = cy * sp * cr + sy * cp * sr, z = sy * cp * cr - cy * sp * sr;
    return {w, -x, -y, -z};
}

sa::Vector3 rotate(const sa::Quaternion& q, const sa::Vector3& v)
{
    const double tx = 2.0 * (q.y * v.z - q.z * v.y);
    const double ty = 2.0 * (q.z * v.x - q.x * v.z);
    const double tz = 2.0 * (q.x * v.y - q.y * v.x);
    return {v.x + q.w * tx + (q.y * tz - q.z * ty), v.y + q.w * ty + (q.z * tx - q.x * tz),
            v.z + q.w * tz + (q.x * ty - q.y * tx)};
}

double max_abs3(const sa::Vector3& a, const sa::Vector3& b)
{
    return std::max({std::fabs(a.x - b.x), std::fabs(a.y - b.y), std::fabs(a.z - b.z)});
}

struct Summary {
    std::string name;
    std::string value;
};
std::vector<Summary> summaries;

void note(const std::string& name, const char* fmt, double v)
{
    char buf[128];
    std::snprintf(buf, sizeof(buf), fmt, v);
    summaries.push_back({name, buf});
    std::printf("%-60s %s\n", name.c_str(), buf);
}

// ------------------------------------------------------------------ IMU and mag

void imu(const std::filesystem::path& out)
{
    std::string e;
    auto hk = require(hs::create_hakoniwa_imu_sensor_adapter(setup_for<sa::ImuAdapterSetup>("axis0-hakoniwa.json"), e), e);
    auto gz = require(gz_adapter::create_gz_imu_sensor_adapter(setup_for<sa::ImuAdapterSetup>("axis0-gz-imu.sdf"), e), e);
#ifdef HAKO_SENSORS_WITH_AIRSIM
    auto as = require(as_adapter::create_projectairsim_native_imu_adapter(setup_for<sa::ImuAdapterSetup>("axis0-airsim-imu.json"), e), e);
    double as_gyro_max = 0.0, as_accel_max = 0.0;
    int as_timing = 0;
#endif
    std::ofstream csv(out / "imu.csv");
    csv.precision(17);
    csv << "t,hk_gx,hk_gy,hk_gz,gz_gx,gz_gy,gz_gz,hk_ax,hk_ay,hk_az,gz_ax,gz_ay,gz_az\n";
    double gyro_max = 0.0, accel_max = 0.0;
    int samples = 0, timing_mismatch = 0;
    for (int k = 0; k < 20000; ++k) {
        const double t = 0.001 * k;
        sa::ImuTruth truth{};
        truth.sim_time_ns = static_cast<std::uint64_t>(k) * kStepNs;
        truth.orientation_world_to_sensor = yaw_pitch_roll_world_to_sensor(0.7 * t, 0.3 * std::sin(t), 0.4 * std::cos(0.5 * t));
        truth.angular_velocity = {0.5 * std::sin(7 * t), -0.3 * std::cos(3 * t), 0.7 + 0.1 * std::sin(40 * t)};
        // a hovering vehicle that also sways: specific force = a - g, in the sensor axes
        const sa::Vector3 accel_world{0.8 * std::sin(2 * t), -0.4 * std::cos(5 * t), 0.5 * std::sin(9 * t)};
        truth.specific_force = rotate(truth.orientation_world_to_sensor,
                                      {accel_world.x, accel_world.y, accel_world.z + 9.80665});
        sa::ImuSample a{}, b{};
        if (!hk->step(truth, a, e) || !gz->step(truth, b, e)) fail(e);
        timing_mismatch += (a.gyro.updated != b.gyro.updated) || (a.accel.updated != b.accel.updated);
#ifdef HAKO_SENSORS_WITH_AIRSIM
        sa::ImuSample c{};
        if (!as->step(truth, c, e)) fail(e);
        as_timing += (a.gyro.updated != c.gyro.updated) || (a.accel.updated != c.accel.updated);
        if (a.gyro.updated) {
            as_gyro_max = std::max(as_gyro_max, max_abs3(a.gyro.value, c.gyro.value));
            as_accel_max = std::max(as_accel_max, max_abs3(a.accel.value, c.accel.value));
        }
#endif
        if (a.gyro.updated) {
            ++samples;
            gyro_max = std::max(gyro_max, max_abs3(a.gyro.value, b.gyro.value));
            accel_max = std::max(accel_max, max_abs3(a.accel.value, b.accel.value));
            csv << t << ',' << a.gyro.value.x << ',' << a.gyro.value.y << ',' << a.gyro.value.z << ','
                << b.gyro.value.x << ',' << b.gyro.value.y << ',' << b.gyro.value.z << ',' << a.accel.value.x << ','
                << a.accel.value.y << ',' << a.accel.value.z << ',' << b.accel.value.x << ',' << b.accel.value.y << ','
                << b.accel.value.z << '\n';
        }
    }
    note("imu: samples (250 Hz, 20 s)", "%.0f", samples);
    note("imu: steps whose measured flag differs", "%.0f", timing_mismatch);
    note("imu: gyro max |hk - gz| [rad/s]", "%.3e", gyro_max);
    note("imu: accel max |hk - gz| [m/s^2]", "%.3e", accel_max);
#ifdef HAKO_SENSORS_WITH_AIRSIM
    note("imu: steps whose measured flag differs (hk vs airsim)", "%.0f", as_timing);
    note("imu: gyro max |hk - airsim| [rad/s]", "%.3e", as_gyro_max);
    note("imu: accel max |hk - airsim| [m/s^2]", "%.3e", as_accel_max);
#endif
}

void mag(const std::filesystem::path& out)
{
    std::string e;
    auto hk = require(hs::create_hakoniwa_mag_sensor_adapter(setup_for<sa::MagAdapterSetup>("axis0-hakoniwa.json"), e), e);
    auto gz = require(gz_adapter::create_gz_mag_sensor_adapter(setup_for<sa::MagAdapterSetup>("axis0-gz-mag.sdf"), e), e);
#ifdef HAKO_SENSORS_WITH_AIRSIM
    auto as = require(as_adapter::create_projectairsim_native_mag_adapter(setup_for<sa::MagAdapterSetup>("axis0-airsim-mag.json"), e), e);
    double as_max = 0.0;
    int as_timing = 0;
#endif
    // Earth field at the location (53045 nT, declination 15.011 deg east, inclination 68.984 deg down), NWU.
    const double h = 53045.1 * std::cos(68.984 * kDegToRad);
    const sa::Vector3 world{h * std::cos(15.011 * kDegToRad), -h * std::sin(15.011 * kDegToRad),
                            -53045.1 * std::sin(68.984 * kDegToRad)};
    std::ofstream csv(out / "mag.csv");
    csv.precision(17);
    csv << "t,hk_x,hk_y,hk_z,gz_x,gz_y,gz_z\n";
    double max_diff = 0.0;
    int samples = 0, timing_mismatch = 0;
    for (int k = 0; k < 20000; ++k) {
        const double t = 0.001 * k;
        sa::MagTruth truth{};
        truth.sim_time_ns = static_cast<std::uint64_t>(k) * kStepNs;
        truth.orientation_world_to_sensor = yaw_pitch_roll_world_to_sensor(0.7 * t, 0.3 * std::sin(t), 0.4 * std::cos(0.5 * t));
        truth.magnetic_field_world = world;
        truth.magnetic_field = rotate(truth.orientation_world_to_sensor, world);
        sa::MagSample a{}, b{};
        if (!hk->step(truth, a, e) || !gz->step(truth, b, e)) fail(e);
        timing_mismatch += a.updated != b.updated;
#ifdef HAKO_SENSORS_WITH_AIRSIM
        sa::MagSample c{};
        if (!as->step(truth, c, e)) fail(e);
        as_timing += a.updated != c.updated;
        if (a.updated) as_max = std::max(as_max, max_abs3(a.value, c.value));
#endif
        if (a.updated) {
            ++samples;
            max_diff = std::max(max_diff, max_abs3(a.value, b.value));
            csv << t << ',' << a.value.x << ',' << a.value.y << ',' << a.value.z << ',' << b.value.x << ','
                << b.value.y << ',' << b.value.z << '\n';
        }
    }
    note("mag: samples (50 Hz, 20 s)", "%.0f", samples);
    note("mag: steps whose measured flag differs", "%.0f", timing_mismatch);
    note("mag: max |hk - gz| [nT]", "%.3e", max_diff);
#ifdef HAKO_SENSORS_WITH_AIRSIM
    note("mag: steps whose measured flag differs (hk vs airsim)", "%.0f", as_timing);
    note("mag: max |hk - airsim| [nT]", "%.3e", as_max);
#endif
}

// ------------------------------------------------------------------------ baro

// Independent reference: US 1976 standard atmosphere (troposphere) with
// geopotential height, g0 = 9.80665, R* = 8.31432, M = 0.0289644.
double us1976_pressure(double z)
{
    const double r0 = 6356766.0;
    const double h = r0 * z / (r0 + z);
    return 101325.0 * std::pow(288.15 / (288.15 - 0.0065 * h), -9.80665 * 0.0289644 / (8.31432 * 0.0065));
}

void baro(const std::filesystem::path& out)
{
    std::string e;
    auto hk = require(hs::create_hakoniwa_baro_sensor_adapter(setup_for<sa::BaroAdapterSetup>("axis0-hakoniwa.json"), e), e);
    auto gz = require(gz_adapter::create_gz_baro_sensor_adapter(setup_for<sa::BaroAdapterSetup>("axis0-gz-baro.sdf"), e), e);
#ifdef HAKO_SENSORS_WITH_AIRSIM
    auto as = require(as_adapter::create_projectairsim_native_baro_adapter(setup_for<sa::BaroAdapterSetup>("axis0-airsim-baro.json"), e), e);
    double max_as_ref = 0.0, max_as_ref_alt = 0.0, max_as_palt = 0.0;
    int as_timing = 0;
#endif
    std::ofstream csv(out / "baro.csv");
    csv.precision(17);
    csv << "altitude_m,hk_pa,gz_pa,us1976_pa,hk_pressure_altitude_m,airsim_pa\n";
    double max_diff = 0.0, max_diff_alt = 0.0, max_hk_ref = 0.0, max_gz_ref = 0.0, max_palt = 0.0;
    int timing_mismatch = 0;
    // 0 m to 3000 m at 50 m/s (60 s).
    for (int k = 0; k <= 60000; ++k) {
        sa::BaroTruth truth{};
        truth.sim_time_ns = static_cast<std::uint64_t>(k) * kStepNs;
        truth.altitude_m = 0.05 * k;
        sa::BaroSample a{}, b{};
        if (!hk->step(truth, a, e) || !gz->step(truth, b, e)) fail(e);
        timing_mismatch += a.updated != b.updated;
        double airsim_pa = 0.0;
#ifdef HAKO_SENSORS_WITH_AIRSIM
        sa::BaroSample c{};
        if (!as->step(truth, c, e)) fail(e);
        as_timing += a.updated != c.updated;
        airsim_pa = c.pressure_pa;
        if (a.updated) {
            const double d = std::fabs(c.pressure_pa - us1976_pressure(truth.altitude_m));
            if (d > max_as_ref) {
                max_as_ref = d;
                max_as_ref_alt = truth.altitude_m;
            }
            max_as_palt = std::max(max_as_palt, std::fabs(c.pressure_altitude_m - truth.altitude_m));
        }
#endif
        if (a.updated) {
            const double ref = us1976_pressure(truth.altitude_m);
            if (std::fabs(a.pressure_pa - b.pressure_pa) > max_diff) {
                max_diff = std::fabs(a.pressure_pa - b.pressure_pa);
                max_diff_alt = truth.altitude_m;
            }
            max_hk_ref = std::max(max_hk_ref, std::fabs(a.pressure_pa - ref));
            max_gz_ref = std::max(max_gz_ref, std::fabs(b.pressure_pa - ref));
            max_palt = std::max(max_palt, std::fabs(a.pressure_altitude_m - truth.altitude_m));
            csv << truth.altitude_m << ',' << a.pressure_pa << ',' << b.pressure_pa << ',' << ref << ','
                << a.pressure_altitude_m << ',' << airsim_pa << '\n';
        }
    }
    note("baro: steps whose measured flag differs", "%.0f", timing_mismatch);
    note("baro: max |hk - gz| over 0-3000 m [Pa]", "%.3f", max_diff);
    note("baro: altitude of that max [m]", "%.0f", max_diff_alt);
    note("baro: max |hk - US1976| [Pa]", "%.3f", max_hk_ref);
    note("baro: max |gz - US1976| [Pa]", "%.3f", max_gz_ref);
    note("baro: hk pressure altitude max |error| [m]", "%.3e", max_palt);
#ifdef HAKO_SENSORS_WITH_AIRSIM
    note("baro: steps whose measured flag differs (hk vs airsim)", "%.0f", as_timing);
    note("baro: max |airsim - US1976| [Pa]", "%.3f", max_as_ref);
    note("baro: altitude of that max [m]", "%.0f", max_as_ref_alt);
    note("baro: airsim pressure altitude max |error| [m]", "%.3e", max_as_palt);
#endif
}

// ------------------------------------------------------------------------- GPS

void gps(const std::filesystem::path& out)
{
    std::string e;
    auto hs_setup = setup_for<sa::GpsAdapterSetup>("axis0-hakoniwa.json");
    hs_setup.origin = {kLat0, kLon0, kAlt0};
    auto gz_setup = setup_for<sa::GpsAdapterSetup>("axis0-gz-gps.sdf");
    gz_setup.origin = hs_setup.origin;
    auto hk = require(hs::create_hakoniwa_gps_sensor_adapter(hs_setup, e), e);
    auto gz = require(gz_adapter::create_gz_gps_sensor_adapter(gz_setup, e), e);
#ifdef HAKO_SENSORS_WITH_AIRSIM
    auto as_setup = setup_for<sa::GpsAdapterSetup>("axis0-airsim-gps.json");
    as_setup.origin = hs_setup.origin;
    auto as = require(as_adapter::create_projectairsim_native_gps_adapter(as_setup, e), e);
    double as_max_h = 0.0, as_max_v = 0.0, as_max_vel = 0.0, as_hk_n = 0.0, as_hk_e = 0.0, as_hk_v = 0.0;
    int as_timing = 0, as_not_3d = 0;
#endif

    // Local to geodetic as gz-sim does for NavSat (world ENU, heading 0).
    const gz::math::SphericalCoordinates sc(gz::math::SphericalCoordinates::EARTH_WGS84,
                                            gz::math::Angle(kLat0 * kDegToRad), gz::math::Angle(kLon0 * kDegToRad),
                                            kAlt0, gz::math::Angle(0.0));
    const double m_per_deg_lat = 111132.954 - 559.822 * std::cos(2 * kLat0 * kDegToRad);
    const double m_per_deg_lon = 111412.84 * std::cos(kLat0 * kDegToRad);

    std::ofstream csv(out / "gps.csv");
    csv << "distance_m,north_err_m,east_err_m,up_err_m,hk_lat,hk_lon,hk_alt,gz_lat,gz_lon,gz_alt\n";
    double max_h = 0.0, max_v = 0.0, max_vel = 0.0;
    int timing_mismatch = 0;
    // From the origin to 10 km north-west at 100 m/s, climbing 50 m (100 s).
    const double speed = 100.0;
    for (int k = 0; k <= 100000; ++k) {
        const double t = 0.001 * k;
        sa::GpsTruth truth{};
        truth.sim_time_ns = static_cast<std::uint64_t>(k) * kStepNs;
        const double d = speed * t;
        truth.position_nwu = {d / std::sqrt(2.0), d / std::sqrt(2.0), 0.5 * t};
        truth.velocity_nwu = {speed / std::sqrt(2.0), speed / std::sqrt(2.0), 0.5};
        const auto enu = gz::math::CoordinateVector3::Metric(-truth.position_nwu.y, truth.position_nwu.x,
                                                             truth.position_nwu.z);
        const auto geo = sc.SphericalFromLocalPosition(enu);
        if (!geo) fail("SphericalFromLocalPosition failed");
        truth.position = {geo->Lat()->Degree(), geo->Lon()->Degree(), *geo->Z()};

        sa::GpsSample a{}, b{};
        if (!hk->step(truth, a, e) || !gz->step(truth, b, e)) fail(e);
        timing_mismatch += a.updated != b.updated;
#ifdef HAKO_SENSORS_WITH_AIRSIM
        sa::GpsSample c{};
        if (!as->step(truth, c, e)) fail(e);
        as_timing += a.updated != c.updated;
        if (c.updated) {
            // core_sim maps the local position itself (a sphere around the home point; its GPS message
            // carries degrees as 32-bit floats), as Hakoniwa does; compared with gz-math's WGS84 mapping.
            as_max_h = std::max(as_max_h, std::hypot((c.position.latitude_deg - b.position.latitude_deg) * m_per_deg_lat,
                                                     (c.position.longitude_deg - b.position.longitude_deg) * m_per_deg_lon));
            as_max_v = std::max(as_max_v, std::fabs(c.position.altitude_m - b.position.altitude_m));
            if (a.updated) {
                as_hk_n = std::max(as_hk_n, std::fabs(c.position.latitude_deg - a.position.latitude_deg) * m_per_deg_lat);
                as_hk_e = std::max(as_hk_e, std::fabs(c.position.longitude_deg - a.position.longitude_deg) * m_per_deg_lon);
                as_hk_v = std::max(as_hk_v, std::fabs(c.position.altitude_m - a.position.altitude_m));
            }
            as_max_vel = std::max(as_max_vel, max_abs3(c.velocity_nwu, b.velocity_nwu));
            as_not_3d += c.fix_type != 3;
        }
#endif
        if (a.updated) {
            const double n = (a.position.latitude_deg - b.position.latitude_deg) * m_per_deg_lat;
            const double ea = (a.position.longitude_deg - b.position.longitude_deg) * m_per_deg_lon;
            const double u = a.position.altitude_m - b.position.altitude_m;
            max_h = std::max(max_h, std::hypot(n, ea));
            max_v = std::max(max_v, std::fabs(u));
            max_vel = std::max(max_vel, max_abs3(a.velocity_nwu, b.velocity_nwu));
            char line[512];
            std::snprintf(line, sizeof(line), "%.1f,%.6f,%.6f,%.6f,%.10f,%.10f,%.6f,%.10f,%.10f,%.6f\n", d, n, ea, u,
                          a.position.latitude_deg, a.position.longitude_deg, a.position.altitude_m,
                          b.position.latitude_deg, b.position.longitude_deg, b.position.altitude_m);
            csv << line;
        }
    }
    note("gps: steps whose measured flag differs", "%.0f", timing_mismatch);
    note("gps: max horizontal |hk - gz| within 10 km [m]", "%.3f", max_h);
    note("gps: max vertical |hk - gz| within 10 km [m]", "%.3f", max_v);
    note("gps: max velocity |hk - gz| [m/s]", "%.3e", max_vel);
#ifdef HAKO_SENSORS_WITH_AIRSIM
    note("gps: steps whose measured flag differs (hk vs airsim)", "%.0f", as_timing);
    note("gps: max horizontal |airsim - gz| within 10 km [m]", "%.3f", as_max_h);
    note("gps: max vertical |airsim - gz| within 10 km [m]", "%.3f", as_max_v);
    note("gps: max north |airsim - hk| within 10 km [m]", "%.3f", as_hk_n);
    note("gps: max east |airsim - hk| within 10 km [m]", "%.3f", as_hk_e);
    note("gps: max vertical |airsim - hk| within 10 km [m]", "%.3f", as_hk_v);
    note("gps: max velocity |airsim - gz| [m/s]", "%.3e", as_max_vel);
    note("gps: airsim samples without a 3D fix", "%.0f", as_not_3d);
#endif
}

// ---------------------------------------------------------------------- timing

// A sensor period that is not a whole number of physics steps: 250 Hz (4 ms) on a 3 ms step.
void timing(const std::filesystem::path& out)
{
    std::string e;
    const std::uint64_t step = 3000000;
    auto hk = require(hs::create_hakoniwa_imu_sensor_adapter(setup_for<sa::ImuAdapterSetup>("axis0-hakoniwa.json", step), e), e);
    auto gz = require(gz_adapter::create_gz_imu_sensor_adapter(setup_for<sa::ImuAdapterSetup>("axis0-gz-imu.sdf", step), e), e);
    std::ofstream csv(out / "timing.csv");
    csv << "t_ms,hk_measured,gz_measured\n";
    int hk_count = 0, gz_count = 0;
    for (int k = 0; k < 1000; ++k) {
        sa::ImuTruth truth{};
        truth.sim_time_ns = static_cast<std::uint64_t>(k) * step;
        truth.specific_force = {0, 0, 9.80665};
        sa::ImuSample a{}, b{};
        if (!hk->step(truth, a, e) || !gz->step(truth, b, e)) fail(e);
        hk_count += a.gyro.updated;
        gz_count += b.gyro.updated;
        if (k < 40) csv << 3 * k << ',' << a.gyro.updated << ',' << b.gyro.updated << '\n';
    }
    note("timing (4 ms on a 3 ms step, 3 s): hk samples", "%.0f", hk_count);
    note("timing (4 ms on a 3 ms step, 3 s): gz samples", "%.0f", gz_count);
#ifdef HAKO_SENSORS_WITH_AIRSIM
    auto as = as_adapter::create_projectairsim_native_imu_adapter(setup_for<sa::ImuAdapterSetup>("axis0-airsim-imu.json", step), e);
    note("timing (4 ms on a 3 ms step): airsim accepts the setting (1) or rejects it (0)", "%.0f", as ? 1.0 : 0.0);
    if (!as) std::printf("  airsim: %s\n", e.c_str());
#endif
}

}  // namespace

int main(int argc, char** argv)
{
    if (argc != 2) {
        std::fprintf(stderr, "usage: %s OUT_DIR\n", argv[0]);
        return 2;
    }
    const std::filesystem::path out(argv[1]);
    std::filesystem::create_directories(out);
    imu(out);
    mag(out);
    baro(out);
    gps(out);
    timing(out);
    std::ofstream md(out / "summary.csv");
    md << "item,value\n";
    for (const auto& s : summaries) md << '"' << s.name << "\"," << s.value << '\n';
    return 0;
}
