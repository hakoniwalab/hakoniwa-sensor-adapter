// Axis 3 of docs/evaluation/drone/evaluation-axes.md: does the same seed give the same sequence?
//
// Usage:
//   compare_axis3_repeatability OUT_DIR          all in-process cases, writes OUT_DIR/axis3.csv
//   compare_axis3_repeatability baro-process ALT  one Gazebo barometer at ALT metres for 10 s; prints the
//                                                 largest error against its own no-noise value (used to run
//                                                 two processes at once on one PC)
#include <unistd.h>

#include <atomic>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <functional>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "hakoniwa/sensor_adapter/gazebo/gz_baro_sensor_adapter.hpp"
#include "hakoniwa/sensor_adapter/gazebo/gz_imu_sensor_adapter.hpp"
#ifdef HAKO_SENSORS_WITH_AIRSIM
#include "hakoniwa/sensor_adapter/airsim/projectairsim_native_imu_adapter.hpp"
#endif
#include "hakoniwa_entry.hpp"

namespace sa = hakoniwa::sensor_adapter;
namespace gza = hakoniwa::sensor_adapter::gazebo;
namespace fs = std::filesystem;

namespace {

constexpr std::uint64_t kStepNs = 1000000;
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

using Factory = std::function<std::unique_ptr<sa::IImuSensorAdapter>(std::uint64_t seed)>;

// The first n gyro x samples of an IMU at rest.
std::vector<double> sequence(sa::IImuSensorAdapter& imu, int n)
{
    std::string e;
    std::vector<double> v;
    sa::ImuTruth t{};
    t.specific_force = {0, 0, 9.80665};
    for (std::uint64_t k = 0; static_cast<int>(v.size()) < n; ++k) {
        t.sim_time_ns = k * kStepNs;
        sa::ImuSample s{};
        if (!imu.step(t, s, e)) fail(e);
        if (s.gyro.updated) v.push_back(s.gyro.value.x);
    }
    return v;
}

void row(const std::string& impl, const std::string& item, bool same)
{
    table << impl << ',' << item << ',' << (same ? "same" : "different") << '\n';
    std::printf("%-8s %-58s %s\n", impl.c_str(), item.c_str(), same ? "same" : "DIFFERENT");
    std::fflush(stdout);
}

void cases(const std::string& impl, const Factory& make)
{
    const int n = 500;
    std::string e;
    const auto reference = sequence(*make(42), n);

    {  // the same instance after reset
        auto imu = make(42);
        sequence(*imu, n);
        imu->reset(e);
        row(impl, "the same instance after reset", sequence(*imu, n) == reference);
    }
    {  // a new instance with the same seed
        row(impl, "a new instance with the same seed", sequence(*make(42), n) == reference);
    }
    {  // another sensor created and stepped alongside
        auto a = make(42);
        auto b = make(43);
        std::vector<double> va;
        sa::ImuTruth t{};
        t.specific_force = {0, 0, 9.80665};
        for (std::uint64_t k = 0; static_cast<int>(va.size()) < n; ++k) {
            t.sim_time_ns = k * kStepNs;
            sa::ImuSample sa_{}, sb{};
            if (!a->step(t, sa_, e) || !b->step(t, sb, e)) fail(e);
            if (sa_.gyro.updated) va.push_back(sa_.gyro.value.x);
        }
        row(impl, "with another sensor stepped alongside", va == reference);
    }
    {  // another sensor reset in the middle
        auto a = make(42);
        auto b = make(43);
        std::vector<double> va;
        sa::ImuTruth t{};
        t.specific_force = {0, 0, 9.80665};
        for (std::uint64_t k = 0; static_cast<int>(va.size()) < n; ++k) {
            t.sim_time_ns = k * kStepNs;
            sa::ImuSample s{};
            if (!a->step(t, s, e)) fail(e);
            if (s.gyro.updated) va.push_back(s.gyro.value.x);
            if (k == 200) b->reset(e);
        }
        row(impl, "with another sensor reset in the middle", va == reference);
    }
    {  // two instances on two threads at the same time
        auto a = make(42);
        auto b = make(42);
        std::vector<double> va, vb;
        std::thread ta([&] { va = sequence(*a, n * 20); });
        std::thread tb([&] { vb = sequence(*b, n * 20); });
        ta.join();
        tb.join();
        auto c = make(42);
        const auto alone = sequence(*c, n * 20);
        row(impl, "two instances on two threads at the same time", va == alone && vb == alone);
    }
    // a fingerprint of the sequence, to compare across operating systems
    double sum = 0;
    for (double x : reference) sum += x;
    char line[256];
    std::snprintf(line, sizeof(line), "%s,fingerprint (sum of the first %d samples, seed 42),%.17g\n", impl.c_str(), n,
                  sum);
    table << line;
    std::printf("%-8s fingerprint %.17g\n", impl.c_str(), sum);
}

int baro_process(double altitude)
{
    out_dir = fs::temp_directory_path() / ("hako-axis3-" + std::to_string(::getpid()));
    std::string e;
    sa::BaroAdapterSetup s{};
    s.config_path = write_file("baro.sdf",
                               "<?xml version=\"1.0\"?><sdf version=\"1.11\"><model name=\"v\"><link name=\"b\"><sensor "
                               "name=\"baro\" type=\"air_pressure\"><update_rate>50</update_rate></sensor></link></model></sdf>");
    s.physics_step_ns = kStepNs;
    auto baro = gza::create_gz_baro_sensor_adapter(s, e);
    if (!baro) fail(e);
    double first = 0, worst = 0;
    sa::BaroTruth t{};
    t.altitude_m = altitude;
    for (std::uint64_t k = 0; k < 10000; ++k) {
        t.sim_time_ns = k * kStepNs;
        sa::BaroSample out{};
        if (!baro->step(t, out, e)) fail(e);
        if (k == 0) first = out.pressure_pa;
        worst = std::max(worst, std::fabs(out.pressure_pa - first));
    }
    fs::remove_all(out_dir);
    std::printf("%.6f\n", worst);
    return 0;
}

}  // namespace

int main(int argc, char** argv)
{
    if (argc == 3 && std::string(argv[1]) == "baro-process") {
        return baro_process(std::atof(argv[2]));
    }
    if (argc != 2) {
        std::fprintf(stderr, "usage: %s OUT_DIR | baro-process ALT\n", argv[0]);
        return 2;
    }
    out_dir = argv[1];
    fs::create_directories(out_dir);
    table.open(out_dir / "axis3.csv");
    table << "implementation,case,result\n";

    const std::string hk_path = write_file(
        "hk-imu.json",
        "{\"components\":{\"sensors\":{\"gyro\":{\"updatePeriodMsec\":4,\"noise\":0.01,"
        "\"zeroRateOffset\":{\"randomStdDegPerSec\":1.0},"
        "\"drift\":{\"mode\":\"gauss-markov\",\"stddevDegPerSec\":0.05,\"correlationTimeSec\":1}},"
        "\"acc\":{\"updatePeriodMsec\":4,\"noise\":0.05}}}}");
    const std::string gz_noise =
        "<noise type=\"gaussian\"><mean>0</mean><stddev>0.01</stddev><bias_mean>0</bias_mean><bias_stddev>0.0175</bias_stddev>"
        "<dynamic_bias_stddev>0.00123</dynamic_bias_stddev><dynamic_bias_correlation_time>1</dynamic_bias_correlation_time></noise>";
    const std::string gz_path = write_file(
        "gz-imu.sdf", "<?xml version=\"1.0\"?><sdf version=\"1.11\"><model name=\"v\"><link name=\"b\"><sensor name=\"imu\" "
                      "type=\"imu\"><update_rate>250</update_rate><imu><angular_velocity><x>" +
                          gz_noise + "</x><y>" + gz_noise + "</y><z>" + gz_noise +
                          "</z></angular_velocity></imu></sensor></link></model></sdf>");

    auto setup = [](const std::string& path, std::uint64_t seed) {
        sa::ImuAdapterSetup s{};
        s.config_path = path;
        s.vehicle_name = "Drone";
        s.physics_step_ns = kStepNs;
        s.has_random_seed = true;
        s.random_seed = seed;
        return s;
    };
    cases("hakoniwa", [&](std::uint64_t seed) {
        std::string e;
        auto a = hs::create_hakoniwa_imu_sensor_adapter(setup(hk_path, seed), e);
        if (!a) fail(e);
        return a;
    });
#ifdef HAKO_SENSORS_WITH_AIRSIM
    // Project AirSim's core_sim Imu: white noise, a turn-on bias and a random-walk bias.
    const std::string as_path = write_file(
        "as-imu.json",
        "{\"type\":\"imu\",\"update-rate-hz\":250,\"accelerometer\":{\"velocity-random-walk\":0.00316,\"tau\":800,"
        "\"bias-stability\":0.0001,\"turn-on-bias\":\"0 0 0\"},\"gyroscope\":{\"angle-random-walk\":0.000632,\"tau\":500,"
        "\"bias-stability\":0.0001,\"turn-on-bias\":\"0.001 0 0\"}}");
    cases("airsim", [&](std::uint64_t seed) {
        std::string e;
        auto a = hakoniwa::sensor_adapter::airsim::create_projectairsim_native_imu_adapter(setup(as_path, seed), e);
        if (!a) fail(e);
        return a;
    });
#endif
    cases("gazebo", [&](std::uint64_t seed) {
        std::string e;
        auto a = gza::create_gz_imu_sensor_adapter(setup(gz_path, seed), e);
        if (!a) fail(e);
        return a;
    });
    return 0;
}
