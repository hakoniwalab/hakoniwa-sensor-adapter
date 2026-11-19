// Axis 4 of docs/evaluation/drone/evaluation-axes.md: time per physics step (1 ms) of the five sensors,
// for each implementation, with and without errors.
//
// Usage: compare_axis4_speed OUT_DIR hakoniwa|gazebo
// One implementation per process, so that the peak memory (maxrss) belongs to it.
#include <sys/resource.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <memory>
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
using Clock = std::chrono::steady_clock;

namespace {

constexpr std::uint64_t kStepNs = 1000000;
constexpr int kSteps = 60000;  // 60 s of simulation
fs::path out_dir;

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

struct Five {
    std::unique_ptr<sa::IImuSensorAdapter> imu;
    std::unique_ptr<sa::IMagSensorAdapter> mag;
    std::unique_ptr<sa::IBaroSensorAdapter> baro;
    std::unique_ptr<sa::IGpsSensorAdapter> gps;
};

template <typename Setup>
Setup setup(const std::string& path)
{
    Setup s{};
    s.config_path = path;
    s.vehicle_name = "Drone";
    s.physics_step_ns = kStepNs;
    s.has_random_seed = true;
    s.random_seed = 9201;
    return s;
}

Five make_hakoniwa(bool errors)
{
    const std::string e1 = errors ? ",\"noise\":0.01,\"drift\":{\"mode\":\"gauss-markov\",\"stddevDegPerSec\":0.05,\"correlationTimeSec\":1}" : "";
    const std::string e2 = errors ? ",\"noise\":0.05" : "";
    const std::string e3 = errors ? ",\"noise\":100" : "";
    const std::string e4 = errors ? ",\"noise\":0.5" : "";
    const std::string e5 = errors ? ",\"positionNoise\":{\"horizontalM\":0.5,\"verticalM\":1.0},\"velocityNoise\":{\"horizontalMps\":0.1,\"verticalMps\":0.2}" : "";
    const std::string path = write_file(std::string("hk-") + (errors ? "errors" : "plain") + ".json",
        "{\"components\":{\"sensors\":{\"gyro\":{\"updatePeriodMsec\":4" + e1 + "},\"acc\":{\"updatePeriodMsec\":4" + e2 +
            "},\"mag\":{\"updatePeriodMsec\":20" + e3 + "},\"baro\":{\"updatePeriodMsec\":20" + e4 +
            "},\"gps\":{\"updatePeriodMsec\":100" + e5 + "}}}}");
    std::string e;
    Five f;
    f.imu = hs::create_hakoniwa_imu_sensor_adapter(setup<sa::ImuAdapterSetup>(path), e);
    f.mag = hs::create_hakoniwa_mag_sensor_adapter(setup<sa::MagAdapterSetup>(path), e);
    f.baro = hs::create_hakoniwa_baro_sensor_adapter(setup<sa::BaroAdapterSetup>(path), e);
    auto g = setup<sa::GpsAdapterSetup>(path);
    g.origin = {47.641468, -122.140165, 121.321};
    f.gps = hs::create_hakoniwa_gps_sensor_adapter(g, e);
    if (!f.imu || !f.mag || !f.baro || !f.gps) fail(e);
    return f;
}

std::string noise(double sd)
{
    return "<noise type=\"gaussian\"><mean>0</mean><stddev>" + std::to_string(sd) + "</stddev></noise>";
}

std::string sdf(const std::string& type, int rate, const std::string& body)
{
    return "<?xml version=\"1.0\"?><sdf version=\"1.11\"><model name=\"v\"><link name=\"b\"><sensor name=\"s\" type=\"" +
           type + "\"><update_rate>" + std::to_string(rate) + "</update_rate>" + body +
           "</sensor></link></model></sdf>";
}

Five make_gazebo(bool errors)
{
    const std::string tag = errors ? "errors" : "plain";
    auto xyz = [&](double sd) { return errors ? "<x>" + noise(sd) + "</x><y>" + noise(sd) + "</y><z>" + noise(sd) + "</z>" : std::string(); };
    std::string e;
    Five f;
    f.imu = gza::create_gz_imu_sensor_adapter(
        setup<sa::ImuAdapterSetup>(write_file("gz-imu-" + tag + ".sdf",
            sdf("imu", 250, errors ? "<imu><angular_velocity>" + xyz(0.01) + "</angular_velocity><linear_acceleration>" +
                                         xyz(0.05) + "</linear_acceleration></imu>" : ""))), e);
    f.mag = gza::create_gz_mag_sensor_adapter(
        setup<sa::MagAdapterSetup>(write_file("gz-mag-" + tag + ".sdf",
            sdf("magnetometer", 50, errors ? "<magnetometer>" + xyz(1e-7) + "</magnetometer>" : ""))), e);
    f.baro = gza::create_gz_baro_sensor_adapter(
        setup<sa::BaroAdapterSetup>(write_file("gz-baro-" + tag + ".sdf",
            sdf("air_pressure", 50, errors ? "<air_pressure><pressure>" + noise(6.0) + "</pressure></air_pressure>" : ""))), e);
    auto g = setup<sa::GpsAdapterSetup>(write_file("gz-gps-" + tag + ".sdf",
        sdf("navsat", 10, errors ? "<navsat><position_sensing><horizontal>" + noise(4.5e-6) + "</horizontal><vertical>" +
                                       noise(1.0) + "</vertical></position_sensing></navsat>" : "")));
    g.origin = {47.641468, -122.140165, 121.321};
    f.gps = gza::create_gz_gps_sensor_adapter(g, e);
    if (!f.imu || !f.mag || !f.baro || !f.gps) fail(e);
    return f;
}

#ifdef HAKO_SENSORS_WITH_AIRSIM
// Project AirSim's own core_sim sensors (the native adapters of hakoniwa-sensor-adapter-airsim).
Five make_airsim(bool errors)
{
    namespace asa = hakoniwa::sensor_adapter::airsim;
    const std::string tag = errors ? "errors" : "plain";
    const double imu_arw = errors ? 0.01 * std::sqrt(0.004) : 0.0;   // white noise 0.01 rad/s at 250 Hz
    const double imu_vrw = errors ? 0.05 * std::sqrt(0.004) : 0.0;   // 0.05 m/s^2 at 250 Hz
    const double imu_bs = errors ? 0.0001 : 0.0;
    std::string e;
    Five f;
    f.imu = asa::create_projectairsim_native_imu_adapter(
        setup<sa::ImuAdapterSetup>(write_file("as-imu-" + tag + ".json",
            "{\"type\":\"imu\",\"update-rate-hz\":250,\"accelerometer\":{\"velocity-random-walk\":" + std::to_string(imu_vrw) +
            ",\"tau\":800,\"bias-stability\":" + std::to_string(imu_bs) + ",\"turn-on-bias\":\"0 0 0\"},\"gyroscope\":{\"angle-random-walk\":" +
            std::to_string(imu_arw) + ",\"tau\":500,\"bias-stability\":" + std::to_string(imu_bs) + ",\"turn-on-bias\":\"0 0 0\"}}")), e);
    f.mag = asa::create_projectairsim_native_mag_adapter(
        setup<sa::MagAdapterSetup>(write_file("as-mag-" + tag + ".json",
            std::string("{\"type\":\"magnetometer\",\"scale-factor\":1.0,\"noise-sigma\":\"") +
            (errors ? "0.001 0.001 0.001" : "0 0 0") + "\",\"noise-bias\":\"0 0 0\",\"update-rate-hz\":50}")), e);
    f.baro = asa::create_projectairsim_native_baro_adapter(
        setup<sa::BaroAdapterSetup>(write_file("as-baro-" + tag + ".json",
            std::string("{\"type\":\"barometer\",\"qnh\":1013.25,\"pressure_factor_sigma\":") + (errors ? "0.0001" : "0") +
            ",\"pressure-factor-tau\":3600,\"uncorrelated_noise_sigma\":" + (errors ? "6.0" : "0") +
            ",\"update-frequency\":50,\"update-latency\":0,\"startup-delay\":0}")), e);
    auto g = setup<sa::GpsAdapterSetup>(write_file("as-gps-" + tag + ".json",
        "{\"type\":\"gps\",\"update-rate-hz\":10,\"eph-time-constant\":0.9,\"epv-time-constant\":0.9,\"eph-initial\":100,"
        "\"epv-initial\":100,\"eph-final\":0.1,\"epv-final\":0.1,\"eph-min-3d\":3,\"eph-min-2d\":4}"));
    g.origin = {47.641468, -122.140165, 121.321};
    f.gps = asa::create_projectairsim_native_gps_adapter(g, e);
    if (!f.imu || !f.mag || !f.baro || !f.gps) fail(e);
    return f;
}
#endif

Five make(const std::string& impl, bool errors)
{
#ifdef HAKO_SENSORS_WITH_AIRSIM
    if (impl == "airsim") return make_airsim(errors);
#endif
    if (impl == "gazebo") return make_gazebo(errors);
    if (impl == "hakoniwa") return make_hakoniwa(errors);
    fail("unknown implementation " + impl);
}

struct Times {
    std::vector<double> imu, mag, baro, gps, all;
};

double pct(std::vector<double> v, double p)
{
    std::sort(v.begin(), v.end());
    return v[static_cast<std::size_t>(p * (v.size() - 1))];
}

Times run(Five& f)
{
    std::string e;
    Times t;
    for (auto* v : {&t.imu, &t.mag, &t.baro, &t.gps, &t.all}) v->reserve(kSteps);
    for (int k = 0; k < kSteps; ++k) {
        const std::uint64_t now = static_cast<std::uint64_t>(k) * kStepNs;
        sa::ImuTruth it{};
        it.sim_time_ns = now;
        it.specific_force = {0, 0, 9.80665};
        it.angular_velocity = {0.1, 0.0, 0.2};
        sa::MagTruth mt{};
        mt.sim_time_ns = now;
        mt.magnetic_field = mt.magnetic_field_world = {18000, -4800, -49500};
        sa::BaroTruth bt{};
        bt.sim_time_ns = now;
        bt.altitude_m = 121.321 + 0.001 * k;
        sa::GpsTruth gt{};
        gt.sim_time_ns = now;
        gt.position = {47.641468, -122.140165, 121.321};
        sa::ImuSample is{};
        sa::MagSample ms{};
        sa::BaroSample bs{};
        sa::GpsSample gs{};
        const auto t0 = Clock::now();
        if (!f.imu->step(it, is, e)) fail(e);
        const auto t1 = Clock::now();
        if (!f.mag->step(mt, ms, e)) fail(e);
        const auto t2 = Clock::now();
        if (!f.baro->step(bt, bs, e)) fail(e);
        const auto t3 = Clock::now();
        if (!f.gps->step(gt, gs, e)) fail(e);
        const auto t4 = Clock::now();
        auto us = [](Clock::time_point a, Clock::time_point b) {
            return std::chrono::duration<double, std::micro>(b - a).count();
        };
        t.imu.push_back(us(t0, t1));
        t.mag.push_back(us(t1, t2));
        t.baro.push_back(us(t2, t3));
        t.gps.push_back(us(t3, t4));
        t.all.push_back(us(t0, t4));
    }
    return t;
}

}  // namespace

int main(int argc, char** argv)
{
    if (argc != 3) {
        std::fprintf(stderr, "usage: %s OUT_DIR hakoniwa|gazebo|airsim\n", argv[0]);
        return 2;
    }
    out_dir = argv[1];
    fs::create_directories(out_dir);
    const std::string impl = argv[2];
    std::ofstream csv(out_dir / ("axis4-" + impl + ".csv"));
    csv << "implementation,errors,sensor,median_us,p99_us,mean_us,total_s_for_60s\n";
    for (bool errors : {false, true}) {
        Five f = make(impl, errors);
        run(f);  // warm up
        Five g = make(impl, errors);
        const Times t = run(g);
        const std::pair<const char*, const std::vector<double>*> parts[] = {
            {"imu", &t.imu}, {"mag", &t.mag}, {"baro", &t.baro}, {"gps", &t.gps}, {"all five", &t.all}};
        for (const auto& [name, v] : parts) {
            double sum = 0;
            for (double x : *v) sum += x;
            char line[256];
            std::snprintf(line, sizeof(line), "%s,%s,%s,%.3f,%.3f,%.3f,%.4f\n", impl.c_str(), errors ? "on" : "off", name,
                          pct(*v, 0.5), pct(*v, 0.99), sum / v->size(), sum * 1e-6);
            csv << line;
            std::printf("%s", line);
        }
    }
    struct rusage ru{};
    getrusage(RUSAGE_SELF, &ru);
#ifdef __APPLE__
    const double maxrss_mb = ru.ru_maxrss / 1e6;  // bytes on macOS
#else
    const double maxrss_mb = ru.ru_maxrss / 1e3;  // kilobytes on Linux
#endif
    csv << impl << ",,peak memory (maxrss),,,," << maxrss_mb << " MB\n";
    std::printf("%s peak memory %.1f MB\n", impl.c_str(), maxrss_mb);
    return 0;
}
