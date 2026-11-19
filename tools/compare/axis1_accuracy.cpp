// Axis 1 of docs/evaluation/drone/evaluation-axes.md: one error setting at a time, the same meaning in
// both implementations (drone_config for Hakoniwa, SDFormat for Gazebo), compared by
// statistics of (output - truth) against values computed from the settings.
//
// Usage: compare_axis1_accuracy OUT_DIR
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <functional>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

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
namespace gza = hakoniwa::sensor_adapter::gazebo;
namespace fs = std::filesystem;

namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kDeg = kPi / 180.0;
constexpr double kG = 9.80665;
constexpr std::uint64_t kStepNs = 1000000;
constexpr double kLat0 = 47.641468, kLon0 = -122.140165, kAlt0 = 121.321;

fs::path out_dir;
std::ofstream table;

[[noreturn]] void fail(const std::string& what)
{
    std::fprintf(stderr, "error: %s\n", what.c_str());
    std::exit(1);
}

std::string write_file(const std::string& name, const std::string& text)
{
    const fs::path p = out_dir / "config" / name;
    fs::create_directories(p.parent_path());
    std::ofstream(p) << text;
    return p.string();
}

// --- statistics -------------------------------------------------------------

struct Stats {
    double mean{0}, sd{0}, autocorr{0}, on_grid{0};
    std::size_t n{0};
};

Stats stats(const std::vector<double>& v, int lag = 0, double grid = 0.0, double offset = 0.0)
{
    Stats s{};
    s.n = v.size();
    if (v.empty()) return s;
    double sum = 0, sum2 = 0;
    for (double x : v) sum += x;
    s.mean = sum / v.size();
    for (double x : v) sum2 += (x - s.mean) * (x - s.mean);
    s.sd = std::sqrt(sum2 / v.size());
    if (lag > 0 && static_cast<std::size_t>(lag) < v.size() && s.sd > 0) {
        double c = 0;
        for (std::size_t i = lag; i < v.size(); ++i) c += (v[i] - s.mean) * (v[i - lag] - s.mean);
        s.autocorr = c / (v.size() - lag) / (s.sd * s.sd);
    }
    if (grid > 0) {
        std::size_t hit = 0;
        for (double x : v) {
            const double q = (x + offset) / grid;
            hit += std::fabs(q - std::round(q)) < 1e-6;
        }
        s.on_grid = static_cast<double>(hit) / v.size();
    }
    return s;
}

void row(const std::string& sensor, const std::string& item, const std::string& stat, const std::string& expected,
         double hk, double gz, const std::string& unit)
{
    char line[512];
    std::snprintf(line, sizeof(line), "%s,\"%s\",%s,%s,%.6g,%.6g,%s\n", sensor.c_str(), item.c_str(), stat.c_str(),
                  expected.c_str(), hk, gz, unit.c_str());
    table << line;
    std::printf("%-6s %-28s %-24s expected %-14s hk %-12.6g gz %-12.6g %s\n", sensor.c_str(), item.c_str(),
                stat.c_str(), expected.c_str(), hk, gz, unit.c_str());
    std::fflush(stdout);
}

std::string num(double v)
{
    char b[64];
    std::snprintf(b, sizeof(b), "%.6g", v);
    return b;
}

// --- SDFormat helpers ---------------------------------------------------------

struct GzNoise {
    double stddev{0}, bias_mean{0}, bias_stddev{0}, dyn_stddev{0}, dyn_tau{0}, precision{0};
    std::string xml() const
    {
        std::ostringstream o;
        o << "<noise type=\"" << (precision > 0 ? "gaussian_quantized" : "gaussian") << "\"><mean>0</mean>"
          << "<stddev>" << stddev << "</stddev><bias_mean>" << bias_mean << "</bias_mean><bias_stddev>"
          << bias_stddev << "</bias_stddev>";
        if (dyn_stddev > 0) {
            o << "<dynamic_bias_stddev>" << dyn_stddev << "</dynamic_bias_stddev><dynamic_bias_correlation_time>"
              << dyn_tau << "</dynamic_bias_correlation_time>";
        }
        if (precision > 0) o << "<precision>" << precision << "</precision>";
        o << "</noise>";
        return o.str();
    }
};

std::string sdf(const std::string& type, double rate, const std::string& body)
{
    std::ostringstream o;
    o << "<?xml version=\"1.0\"?><sdf version=\"1.11\"><model name=\"vehicle\"><link name=\"body\"><sensor name=\"s\" type=\""
      << type << "\"><update_rate>" << rate << "</update_rate>" << body << "</sensor></link></model></sdf>";
    return o.str();
}

std::string xyz(const GzNoise& n)
{
    return "<x>" + n.xml() + "</x><y>" + n.xml() + "</y><z>" + n.xml() + "</z>";
}

std::string hk_config(const std::string& sensor, const std::string& body)
{
    return "{\"components\":{\"sensors\":{\"" + sensor + "\":" + body + "}}}";
}

template <typename Setup>
Setup setup(const std::string& path, std::uint64_t seed)
{
    Setup s{};
    s.config_path = path;
    s.vehicle_name = "Drone";
    s.physics_step_ns = kStepNs;
    s.has_random_seed = true;
    s.random_seed = seed;
    return s;
}

// --- one run per sensor: error series of the x axis (or a chosen component) -----

// IMU at rest, level: truth gyro 0, specific force +g on z. Returns gyro x and accel x errors.
void run_imu(sa::IImuSensorAdapter& imu, int samples, std::vector<double>& gyro_x, std::vector<double>& accel_x)
{
    std::string e;
    sa::ImuTruth t{};
    t.specific_force = {0, 0, kG};
    for (std::uint64_t k = 0; static_cast<int>(gyro_x.size()) < samples; ++k) {
        t.sim_time_ns = k * kStepNs;
        sa::ImuSample s{};
        if (!imu.step(t, s, e)) fail(e);
        if (s.gyro.updated) {
            gyro_x.push_back(s.gyro.value.x);
            accel_x.push_back(s.accel.value.x);
        }
    }
}

std::unique_ptr<sa::IImuSensorAdapter> hk_imu(const std::string& gyro, const std::string& acc, std::uint64_t seed)
{
    std::string e;
    const auto path = write_file("hk-imu-" + std::to_string(seed) + ".json",
                                 "{\"components\":{\"sensors\":{\"gyro\":" + gyro + ",\"acc\":" + acc + "}}}");
    auto a = hs::create_hakoniwa_imu_sensor_adapter(setup<sa::ImuAdapterSetup>(path, seed), e);
    if (!a) fail(e);
    return a;
}

std::unique_ptr<sa::IImuSensorAdapter> gz_imu(const GzNoise& gyro, const GzNoise& acc, std::uint64_t seed)
{
    std::string e;
    const auto path = write_file("gz-imu-" + std::to_string(seed) + ".sdf",
                                 sdf("imu", 250, "<imu><angular_velocity>" + xyz(gyro) +
                                                     "</angular_velocity><linear_acceleration>" + xyz(acc) +
                                                     "</linear_acceleration></imu>"));
    auto a = gza::create_gz_imu_sensor_adapter(setup<sa::ImuAdapterSetup>(path, seed), e);
    if (!a) fail(e);
    return a;
}

void imu_cases()
{
    const int n = 20000;
    const std::string period = "\"updatePeriodMsec\":4";
    // 1. white noise
    {
        std::vector<double> hg, ha, gg, ga;
        run_imu(*hk_imu("{" + period + ",\"noise\":0.01}", "{" + period + ",\"noise\":0.05}", 1), n, hg, ha);
        GzNoise g{0.01}, a{0.05};
        run_imu(*gz_imu(g, a, 1), n, gg, ga);
        row("gyro", "noise 0.01 rad/s", "sd", "0.01", stats(hg).sd, stats(gg).sd, "rad/s");
        row("gyro", "noise 0.01 rad/s", "mean", "0", stats(hg).mean, stats(gg).mean, "rad/s");
        row("accel", "noise 0.05 m/s2", "sd", "0.05", stats(ha).sd, stats(ga).sd, "m/s2");
    }
    // 2. fixed offset over many instances: Gazebo flips the sign with 50 % probability
    {
        const int m = 200;
        std::vector<double> hg, gg;
        for (int i = 0; i < m; ++i) {
            std::vector<double> a, b, c, d;
            run_imu(*hk_imu("{" + period + ",\"zeroRateOffset\":{\"fixedDegPerSec\":[0.5,0,0]}}", "{" + period + "}",
                            100 + i),
                    1, a, b);
            GzNoise g{};
            g.bias_mean = 0.5 * kDeg;
            run_imu(*gz_imu(g, GzNoise{}, 100 + i), 1, c, d);
            hg.push_back(a[0]);
            gg.push_back(c[0]);
        }
        const double expected = 0.5 * kDeg;
        auto positive = [&](const std::vector<double>& v) {
            int p = 0;
            for (double x : v) p += x > 0;
            return static_cast<double>(p) / v.size();
        };
        row("gyro", "fixed offset 0.5 deg/s", "mean over 200 runs", num(expected), stats(hg).mean, stats(gg).mean,
            "rad/s");
        row("gyro", "fixed offset 0.5 deg/s", "share of runs > 0", "1", positive(hg), positive(gg), "");
    }
    // 3. offset drawn at start (1 deg/s) over many instances
    {
        const int m = 400;
        std::vector<double> hg, gg;
        for (int i = 0; i < m; ++i) {
            std::vector<double> a, b, c, d;
            run_imu(*hk_imu("{" + period + ",\"zeroRateOffset\":{\"randomStdDegPerSec\":1.0}}", "{" + period + "}",
                            1000 + i),
                    1, a, b);
            GzNoise g{};
            g.bias_stddev = 1.0 * kDeg;
            run_imu(*gz_imu(g, GzNoise{}, 1000 + i), 1, c, d);
            hg.push_back(a[0]);
            gg.push_back(c[0]);
        }
        row("gyro", "random offset sd 1 deg/s", "sd over 400 runs", num(1.0 * kDeg), stats(hg).sd, stats(gg).sd,
            "rad/s");
    }
    // 4. Gauss-Markov drift: sigma 0.05 deg/s, tau 1 s (Gazebo: dynamic_bias_stddev = sigma sqrt(2/tau))
    {
        const double sigma = 0.05 * kDeg, tau = 1.0;
        std::vector<double> hg, ha, gg, ga;
        run_imu(*hk_imu("{" + period +
                            ",\"drift\":{\"mode\":\"gauss-markov\",\"stddevDegPerSec\":0.05,\"correlationTimeSec\":1}}",
                        "{" + period + "}", 7),
                250000, hg, ha);
        GzNoise g{};
        g.dyn_stddev = sigma * std::sqrt(2.0 / tau);
        g.dyn_tau = tau;
        run_imu(*gz_imu(g, GzNoise{}, 7), 250000, gg, ga);
        const int lag = 250;  // tau at 250 Hz
        row("gyro", "drift GM 0.05 deg/s, tau 1 s", "sd", num(sigma), stats(hg).sd, stats(gg).sd, "rad/s");
        row("gyro", "drift GM 0.05 deg/s, tau 1 s", "autocorr at tau", num(std::exp(-1.0)), stats(hg, lag).autocorr,
            stats(gg, lag).autocorr, "");
    }
    // 5. resolution with noise
    {
        std::vector<double> hg, ha, gg, ga;
        const double r = 0.001;
        char res[64];
        std::snprintf(res, sizeof(res), "%.17g", r / kDeg);
        run_imu(*hk_imu("{" + period + ",\"noise\":0.01,\"resolutionDegPerSec\":" + std::string(res) + "}",
                        "{" + period + "}", 3),
                n, hg, ha);
        GzNoise g{0.01};
        g.precision = r;
        run_imu(*gz_imu(g, GzNoise{}, 3), n, gg, ga);
        row("gyro", "resolution 0.001 rad/s", "share on the grid", "1", stats(hg, 0, r).on_grid,
            stats(gg, 0, r).on_grid, "");
        row("gyro", "resolution 0.001 rad/s", "sd", num(std::sqrt(0.01 * 0.01 + r * r / 12)), stats(hg).sd,
            stats(gg).sd, "rad/s");
    }
}

// --- magnetometer ------------------------------------------------------------------

void run_mag(sa::IMagSensorAdapter& mag, int samples, std::vector<double>& x)
{
    std::string e;
    sa::MagTruth t{};
    t.magnetic_field = {18000, -4800, -49500};
    t.magnetic_field_world = t.magnetic_field;
    for (std::uint64_t k = 0; static_cast<int>(x.size()) < samples; ++k) {
        t.sim_time_ns = k * kStepNs;
        sa::MagSample s{};
        if (!mag.step(t, s, e)) fail(e);
        if (s.updated) x.push_back(s.value.x - 18000);
    }
}

void mag_cases()
{
    std::string e;
    auto hk = [&](const std::string& body, std::uint64_t seed) {
        auto a = hs::create_hakoniwa_mag_sensor_adapter(
            setup<sa::MagAdapterSetup>(write_file("hk-mag-" + std::to_string(seed) + ".json", hk_config("mag", body)),
                                       seed),
            e);
        if (!a) fail(e);
        return a;
    };
    auto gz = [&](const GzNoise& n, std::uint64_t seed) {
        auto a = gza::create_gz_mag_sensor_adapter(
            setup<sa::MagAdapterSetup>(
                write_file("gz-mag-" + std::to_string(seed) + ".sdf", sdf("magnetometer", 50, "<magnetometer>" + xyz(n) + "</magnetometer>")),
                seed),
            e);
        if (!a) fail(e);
        return a;
    };
    const double t = 1e-9;  // tesla per nT
    {
        std::vector<double> h, g;
        run_mag(*hk("{\"updatePeriodMsec\":20,\"noise\":100}", 11), 20000, h);
        run_mag(*gz(GzNoise{100 * t}, 11), 20000, g);
        row("mag", "noise 100 nT", "sd", "100", stats(h).sd, stats(g).sd, "nT");
    }
    {
        std::vector<double> h, g;
        run_mag(*hk("{\"updatePeriodMsec\":20,\"hardIronNT\":[3000,0,0]}", 12), 50, h);
        GzNoise n{};
        n.bias_mean = 3000 * t;
        run_mag(*gz(n, 12), 50, g);
        row("mag", "fixed offset 3000 nT", "mean", "3000", stats(h).mean, stats(g).mean, "nT");
    }
    {
        std::vector<double> h, g;
        run_mag(*hk("{\"updatePeriodMsec\":20,\"drift\":{\"mode\":\"gauss-markov\",\"stddevNT\":100,\"correlationTimeSec\":1}}",
                    13),
                100000, h);
        GzNoise n{};
        n.dyn_stddev = 100 * t * std::sqrt(2.0);
        n.dyn_tau = 1.0;
        run_mag(*gz(n, 13), 100000, g);
        row("mag", "drift GM 100 nT, tau 1 s", "sd", "100", stats(h).sd, stats(g).sd, "nT");
        row("mag", "drift GM 100 nT, tau 1 s", "autocorr at tau", num(std::exp(-1.0)), stats(h, 50).autocorr,
            stats(g, 50).autocorr, "");
    }
    {
        std::vector<double> h, g;
        run_mag(*hk("{\"updatePeriodMsec\":20,\"noise\":100,\"resolutionNT\":15}", 14), 20000, h);
        GzNoise n{100 * t};
        n.precision = 15 * t;
        run_mag(*gz(n, 14), 20000, g);
        row("mag", "resolution 15 nT", "share on the grid", "1", stats(h, 0, 15, 18000).on_grid,
            stats(g, 0, 15, 18000).on_grid, "");
    }
}

// --- barometer (Hakoniwa errors in metres of height, Gazebo in pascals) -------------

double us1976(double z)
{
    const double r0 = 6356766.0, h = r0 * z / (r0 + z);
    return 101325.0 * std::pow(288.15 / (288.15 - 0.0065 * h), -kG * 0.0289644 / (8.31432 * 0.0065));
}

void run_baro(sa::IBaroSensorAdapter& baro, int samples, std::vector<double>& p)
{
    std::string e;
    sa::BaroTruth t{};
    t.altitude_m = kAlt0;
    for (std::uint64_t k = 0; static_cast<int>(p.size()) < samples; ++k) {
        t.sim_time_ns = k * kStepNs;
        sa::BaroSample s{};
        if (!baro.step(t, s, e)) fail(e);
        if (s.updated) p.push_back(s.pressure_pa);
    }
}

void baro_cases()
{
    std::string e;
    auto hk = [&](const std::string& body, std::uint64_t seed) {
        auto a = hs::create_hakoniwa_baro_sensor_adapter(
            setup<sa::BaroAdapterSetup>(write_file("hk-baro-" + std::to_string(seed) + ".json", hk_config("baro", body)),
                                        seed),
            e);
        if (!a) fail(e);
        return a;
    };
    auto gz = [&](const GzNoise& n, std::uint64_t seed) {
        auto a = gza::create_gz_baro_sensor_adapter(
            setup<sa::BaroAdapterSetup>(
                write_file("gz-baro-" + std::to_string(seed) + ".sdf",
                           sdf("air_pressure", 50, "<air_pressure><pressure>" + n.xml() + "</pressure></air_pressure>")),
                seed),
            e);
        if (!a) fail(e);
        return a;
    };
    // Pascals per metre at the location, to give both the same meaning.
    const double pa_per_m = us1976(kAlt0 - 0.5) - us1976(kAlt0 + 0.5);
    {
        std::vector<double> h, g;
        run_baro(*hk("{\"updatePeriodMsec\":20,\"noise\":0.5}", 21), 20000, h);
        run_baro(*gz(GzNoise{0.5 * pa_per_m}, 21), 20000, g);
        row("baro", "noise 0.5 m (" + num(0.5 * pa_per_m) + " Pa)", "sd", num(0.5 * pa_per_m), stats(h).sd,
            stats(g).sd, "Pa");
    }
    {
        std::vector<double> h, g;
        run_baro(*hk("{\"updatePeriodMsec\":20,\"zeroOffset\":{\"fixedM\":2.0}}", 22), 50, h);
        GzNoise n{};
        n.bias_mean = -2.0 * pa_per_m;  // +2 m of height is less pressure
        run_baro(*gz(n, 22), 50, g);
        row("baro", "fixed offset +2 m", "mean - true pressure", num(-2.0 * pa_per_m), stats(h).mean - us1976(kAlt0),
            stats(g).mean - us1976(kAlt0), "Pa");
    }
    {
        std::vector<double> h, g;
        run_baro(*hk("{\"updatePeriodMsec\":20,\"drift\":{\"mode\":\"gauss-markov\",\"stddevM\":0.3,\"correlationTimeSec\":1}}",
                     23),
                 20000, h);
        GzNoise n{};
        n.dyn_stddev = 0.3 * pa_per_m * std::sqrt(2.0);
        n.dyn_tau = 1.0;
        run_baro(*gz(n, 23), 20000, g);
        row("baro", "drift GM 0.3 m, tau 1 s", "sd", num(0.3 * pa_per_m), stats(h).sd, stats(g).sd, "Pa");
    }
    {
        std::vector<double> h, g;
        run_baro(*hk("{\"updatePeriodMsec\":20,\"noise\":0.5,\"resolutionPa\":1}", 24), 20000, h);
        GzNoise n{0.5 * pa_per_m};
        n.precision = 1.0;
        run_baro(*gz(n, 24), 20000, g);
        row("baro", "resolution 1 Pa", "share on the grid", "1", stats(h, 0, 1.0).on_grid, stats(g, 0, 1.0).on_grid, "");
    }
}

// --- GPS ---------------------------------------------------------------------------

struct GpsErr {
    std::vector<double> north, east, up, vn;
};

void run_gps(sa::IGpsSensorAdapter& gps, int samples, GpsErr& out)
{
    std::string e;
    const double m_lat = 111132.954 - 559.822 * std::cos(2 * kLat0 * kDeg);
    const double m_lon = 111412.84 * std::cos(kLat0 * kDeg);
    sa::GpsTruth t{};
    t.position = {kLat0, kLon0, kAlt0};
    for (std::uint64_t k = 0; static_cast<int>(out.north.size()) < samples; ++k) {
        t.sim_time_ns = k * kStepNs;
        sa::GpsSample s{};
        if (!gps.step(t, s, e)) fail(e);
        if (s.updated) {
            out.north.push_back((s.position.latitude_deg - kLat0) * m_lat);
            out.east.push_back((s.position.longitude_deg - kLon0) * m_lon);
            out.up.push_back(s.position.altitude_m - kAlt0);
            out.vn.push_back(s.velocity_nwu.x);
        }
    }
}

void gps_cases()
{
    std::string e;
    const double m_lat = 111132.954 - 559.822 * std::cos(2 * kLat0 * kDeg);
    auto hk = [&](const std::string& body, std::uint64_t seed) {
        auto s = setup<sa::GpsAdapterSetup>(write_file("hk-gps-" + std::to_string(seed) + ".json", hk_config("gps", body)),
                                            seed);
        s.origin = {kLat0, kLon0, kAlt0};
        auto a = hs::create_hakoniwa_gps_sensor_adapter(s, e);
        if (!a) fail(e);
        return a;
    };
    auto gz = [&](const GzNoise& h, const GzNoise& v, const GzNoise& vh, const GzNoise& vv, std::uint64_t seed) {
        auto s = setup<sa::GpsAdapterSetup>(
            write_file("gz-gps-" + std::to_string(seed) + ".sdf",
                       sdf("navsat", 10,
                           "<navsat><position_sensing><horizontal>" + h.xml() + "</horizontal><vertical>" + v.xml() +
                               "</vertical></position_sensing><velocity_sensing><horizontal>" + vh.xml() +
                               "</horizontal><vertical>" + vv.xml() + "</vertical></velocity_sensing></navsat>")),
            seed);
        s.origin = {kLat0, kLon0, kAlt0};
        auto a = gza::create_gz_gps_sensor_adapter(s, e);
        if (!a) fail(e);
        return a;
    };
    {
        GpsErr h, g;
        run_gps(*hk("{\"updatePeriodMsec\":100,\"positionNoise\":{\"horizontalM\":0.5,\"verticalM\":1.0},"
                    "\"velocityNoise\":{\"horizontalMps\":0.1,\"verticalMps\":0.2}}",
                    31),
                20000, h);
        // Gazebo horizontal noise is in degrees: 0.5 m of latitude.
        run_gps(*gz(GzNoise{0.5 / m_lat}, GzNoise{1.0}, GzNoise{0.1}, GzNoise{0.2}, 31), 20000, g);
        row("gps", "position noise 0.5 m horiz.", "sd north", "0.5", stats(h.north).sd, stats(g.north).sd, "m");
        row("gps", "position noise 0.5 m horiz.", "sd east", "0.5", stats(h.east).sd, stats(g.east).sd, "m");
        row("gps", "position noise 1.0 m vert.", "sd up", "1", stats(h.up).sd, stats(g.up).sd, "m");
        row("gps", "velocity noise 0.1 m/s", "sd north", "0.1", stats(h.vn).sd, stats(g.vn).sd, "m/s");
    }
    {
        GpsErr h, g;
        run_gps(*hk("{\"updatePeriodMsec\":100,\"drift\":{\"mode\":\"gauss-markov\",\"horizontalM\":0.5,\"verticalM\":0.8,"
                    "\"correlationTimeSec\":1}}",
                    32),
                20000, h);
        GzNoise d{};
        d.dyn_stddev = 0.5 / m_lat * std::sqrt(2.0);
        d.dyn_tau = 1.0;
        run_gps(*gz(d, GzNoise{}, GzNoise{}, GzNoise{}, 32), 20000, g);
        row("gps", "drift GM 0.5 m horiz., tau 1 s", "sd north", "0.5", stats(h.north).sd, stats(g.north).sd, "m");
        row("gps", "drift GM 0.5 m horiz., tau 1 s", "autocorr at tau", num(std::exp(-1.0)), stats(h.north, 10).autocorr,
            stats(g.north, 10).autocorr, "");
    }
}

#ifdef HAKO_SENSORS_WITH_AIRSIM
// ---------------------------------------------------------------- Project AirSim
// Project AirSim's own core_sim sensors (the native adapters of hakoniwa-sensor-adapter-airsim).
// Settings are mapped to the same meaning where Project AirSim has one; the
// others are listed as not expressible.

std::ofstream as_table;

void as_row(const std::string& sensor, const std::string& item, const std::string& stat, const std::string& expected,
            double value, const std::string& unit)
{
    char line[512];
    std::snprintf(line, sizeof(line), "%s,\"%s\",%s,%s,%.6g,%s\n", sensor.c_str(), item.c_str(), stat.c_str(),
                  expected.c_str(), value, unit.c_str());
    as_table << line;
    std::printf("airsim %-6s %-34s %-22s expected %-12s value %-12.6g %s\n", sensor.c_str(), item.c_str(),
                stat.c_str(), expected.c_str(), value, unit.c_str());
    std::fflush(stdout);
}

void as_missing(const std::string& sensor, const std::string& item, const std::string& why)
{
    as_table << sensor << ",\"" << item << "\",not expressible,,,\"" << why << "\"\n";
    std::printf("airsim %-6s %-34s not expressible: %s\n", sensor.c_str(), item.c_str(), why.c_str());
}

std::unique_ptr<sa::IImuSensorAdapter> as_imu(double arw, double vrw, const std::string& gyro_turn_on, double bias_stability,
                                              std::uint64_t seed)
{
    std::string e;
    char text[1024];
    std::snprintf(text, sizeof(text),
                  "{\"type\":\"imu\",\"update-rate-hz\":250,\"accelerometer\":{\"velocity-random-walk\":%.17g,\"tau\":800,"
                  "\"bias-stability\":0,\"turn-on-bias\":\"0 0 0\"},\"gyroscope\":{\"angle-random-walk\":%.17g,\"tau\":500,"
                  "\"bias-stability\":%.17g,\"turn-on-bias\":\"%s\"}}",
                  vrw, arw, bias_stability, gyro_turn_on.c_str());
    auto a = hakoniwa::sensor_adapter::airsim::create_projectairsim_native_imu_adapter(
        setup<sa::ImuAdapterSetup>(write_file("as-imu-" + std::to_string(seed) + ".json", text), seed), e);
    if (!a) fail(e);
    return a;
}

void airsim_cases()
{
    std::string e;
    const double dt = 0.004;  // 250 Hz; the white noise is random_walk / sqrt(dt)
    {
        std::vector<double> g, a;
        run_imu(*as_imu(0.01 * std::sqrt(dt), 0.05 * std::sqrt(dt), "0 0 0", 0.0, 1), 20000, g, a);
        as_row("gyro", "noise 0.01 rad/s", "sd", "0.01", stats(g).sd, "rad/s");
        as_row("accel", "noise 0.05 m/s2", "sd", "0.05", stats(a).sd, "m/s2");
    }
    {
        std::vector<double> g, a;
        char bias[64];
        std::snprintf(bias, sizeof(bias), "%.17g 0 0", 0.5 * kDeg);
        run_imu(*as_imu(0.0, 0.0, bias, 0.0, 2), 50, g, a);
        as_row("gyro", "fixed offset 0.5 deg/s (turn-on-bias)", "mean", num(0.5 * kDeg), stats(g).mean, "rad/s");
    }
    as_missing("gyro", "random offset sd 1 deg/s", "turn-on-bias is a fixed vector");
    as_missing("gyro", "drift GM 0.05 deg/s, tau 1 s", "the bias is a random walk (bias-stability), not Gauss-Markov");
    {
        // the random-walk bias: its spread after T grows as bias_stability / sqrt(tau) * sqrt(T)
        const double bs = 0.001, tau = 500.0, horizon = 100.0;
        std::vector<double> ends;
        for (int i = 0; i < 200; ++i) {
            std::vector<double> g, a;
            run_imu(*as_imu(0.0, 0.0, "0 0 0", bs, 300 + i), static_cast<int>(horizon * 250), g, a);
            ends.push_back(g.back());
        }
        as_row("gyro", "bias random walk 0.001, tau 500 s", "sd after 100 s over 200 runs",
               num(bs / std::sqrt(tau) * std::sqrt(horizon)), stats(ends).sd, "rad/s");
        // core_sim seeds every instance the same, so the runs above are identical. Within one run the
        // change over 100 s has the same spread: 200 back-to-back windows of one 20000 s run.
        std::vector<double> g, acc, changes;
        run_imu(*as_imu(0.0, 0.0, "0 0 0", bs, 299), static_cast<int>(200 * horizon * 250) + 1, g, acc);
        const std::size_t window = static_cast<std::size_t>(horizon * 250);
        for (std::size_t i = 0; i + window < g.size(); i += window) changes.push_back(g[i + window] - g[i]);
        as_row("gyro", "bias random walk 0.001, tau 500 s", "sd of the 100 s change, 200 windows of one run",
               num(bs / std::sqrt(tau) * std::sqrt(horizon)), stats(changes).sd, "rad/s");
    }
    as_missing("gyro", "resolution 0.001 rad/s", "no resolution");

    auto mag = [&](const std::string& sigma_g, const std::string& bias_g, std::uint64_t seed) {
        auto a = hakoniwa::sensor_adapter::airsim::create_projectairsim_native_mag_adapter(
            setup<sa::MagAdapterSetup>(write_file("as-mag-" + std::to_string(seed) + ".json",
                "{\"type\":\"magnetometer\",\"scale-factor\":1.0,\"noise-sigma\":\"" + sigma_g +
                "\",\"noise-bias\":\"" + bias_g + "\",\"update-rate-hz\":50}"), seed), e);
        if (!a) fail(e);
        return a;
    };
    {
        std::vector<double> x;
        run_mag(*mag("0.001 0.001 0.001", "0 0 0", 11), 20000, x);
        as_row("mag", "noise 100 nT (0.001 G)", "sd", "100", stats(x).sd, "nT");
    }
    {
        // noise-bias b: drawn once at start from a normal distribution with mean -b and sd b
        std::vector<double> firsts;
        for (int i = 0; i < 400; ++i) {
            std::vector<double> x;
            run_mag(*mag("0 0 0", "0.03 0.03 0.03", 500 + i), 1, x);
            firsts.push_back(x[0]);
        }
        as_row("mag", "noise-bias 3000 nT (0.03 G)", "mean over 400 runs", "-3000 (mean -b)", stats(firsts).mean, "nT");
        as_row("mag", "noise-bias 3000 nT (0.03 G)", "sd over 400 runs", "3000 (sd b)", stats(firsts).sd, "nT");
    }
    as_missing("mag", "fixed offset 3000 nT", "noise-bias is drawn at start (mean -b, sd b), not fixed");
    as_missing("mag", "drift GM 100 nT, tau 1 s", "no drift");
    as_missing("mag", "resolution 15 nT", "no resolution");

    const double pa_per_m = us1976(kAlt0 - 0.5) - us1976(kAlt0 + 0.5);
    auto baro = [&](double factor_sigma, double factor_tau, double noise_pa, double qnh, std::uint64_t seed) {
        char text[512];
        std::snprintf(text, sizeof(text),
                      "{\"type\":\"barometer\",\"qnh\":%.17g,\"pressure_factor_sigma\":%.17g,\"pressure-factor-tau\":%.17g,"
                      "\"uncorrelated_noise_sigma\":%.17g,\"update-frequency\":50,\"update-latency\":0,\"startup-delay\":0}",
                      qnh, factor_sigma, factor_tau, noise_pa);
        auto a = hakoniwa::sensor_adapter::airsim::create_projectairsim_native_baro_adapter(
            setup<sa::BaroAdapterSetup>(write_file("as-baro-" + std::to_string(seed) + ".json", text), seed), e);
        if (!a) fail(e);
        return a;
    };
    {
        std::vector<double> p;
        run_baro(*baro(0.0, 3600.0, 0.5 * pa_per_m, 1013.25, 21), 20000, p);
        as_row("baro", "noise 0.5 m (" + num(0.5 * pa_per_m) + " Pa)", "sd", num(0.5 * pa_per_m), stats(p).sd, "Pa");
    }
    {
        std::vector<double> p;
        const double offset = -2.0 * pa_per_m;  // +2 m of height
        run_baro(*baro(0.0, 3600.0, 0.0, (101325.0 + offset) / 100.0, 22), 50, p);
        as_row("baro", "fixed offset +2 m (qnh)", "mean - true pressure", num(offset), stats(p).mean - us1976(kAlt0), "Pa");
    }
    {
        // pressure factor (Gauss-Markov on a multiplier): sigma written as the relative spread of 0.3 m
        std::vector<double> p;
        const double rel = 0.3 * pa_per_m / us1976(kAlt0);
        run_baro(*baro(rel, 1.0, 0.0, 1013.25, 23), 20000, p);
        as_row("baro", "drift GM 0.3 m, tau 1 s (pressure factor)", "sd", num(0.3 * pa_per_m), stats(p).sd, "Pa");
        as_row("baro", "drift GM 0.3 m, tau 1 s (pressure factor)", "autocorr at tau", num(std::exp(-1.0)),
               stats(p, 50).autocorr, "");
    }
    as_missing("baro", "resolution 1 Pa", "no resolution");
    as_missing("gps", "position noise / velocity noise / drift", "the GPS passes the truth through without noise");
}
#endif

}  // namespace

int main(int argc, char** argv)
{
    if (argc != 2) {
        std::fprintf(stderr, "usage: %s OUT_DIR\n", argv[0]);
        return 2;
    }
    out_dir = argv[1];
    fs::create_directories(out_dir);
    table.open(out_dir / "axis1.csv");
    table << "sensor,setting,statistic,expected,hakoniwa,gazebo,unit\n";
    imu_cases();
    mag_cases();
    baro_cases();
    gps_cases();
#ifdef HAKO_SENSORS_WITH_AIRSIM
    as_table.open(out_dir / "axis1-airsim.csv");
    as_table << "sensor,setting,statistic,expected,airsim,unit_or_reason\n";
    airsim_cases();
#endif
    return 0;
}
