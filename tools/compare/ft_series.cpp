// ft-series: one force-torque sensor of one implementation, run for a while on a motion written as a formula,
// every physics step to CSV. The measurement cards of the force-torque page are drawn from these files
// (cards_force_torque.py); the comparison tables come from force_torque.cpp.
//
//   ft-series --implementation hakoniwa|gazebo --config FILE --out FILE.csv [--duration S] [--seed N]
//             [--motion rest|sine|ramp|step|file] [--input FILE.csv] [--rate R] [--t0 S] [--other-seed N] [--reset-at S] [--tare-at S]
//             [--every N]
//
// Motions (sensor axes; Fz only unless stated):
//   rest   zero wrench
//   sine   every axis a sine of its own amplitude and frequency plus a constant (as force_torque.cpp)
//   ramp   Fz = --rate N/s × (t − t0) from t0
//   step   Fz = --rate N from t0
//   file   every axis from --input FILE.csv (columns t,fx,fy,fz,tx,ty,tz with a header; t in s, sensor axes),
//          one physics step per row; the step length is taken from the file's first two rows
// --other-seed steps a second sensor of the same configuration (another seed) before each step of this one, as
// another sensor of the same process would. --reset-at calls reset() at that time and restarts the time from 0.
// --tare-at calls tare() at that time. Columns: time (continuous across a reset), updated, fx … tz, true_fx … true_tz,
// flags.
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include "hakoniwa/sensor_adapter/gazebo/gz_force_torque_sensor_adapter.hpp"
#include "hakoniwa_entry.hpp"

namespace sa = hakoniwa::sensor_adapter;

namespace {

std::uint64_t kStepNs = 1000000;  // 1 ms, or the step of an input file
constexpr double kPi = 3.14159265358979323846;

struct Options {
    std::string implementation{"hakoniwa"}, config, out, motion{"rest"}, input;
    double duration{10}, rate{0}, t0{0}, reset_at{-1}, tare_at{-1};
    std::uint64_t seed{1}, other_seed{0}, every{1};
    bool other{false};
};

[[noreturn]] void fail(const std::string& what)
{
    std::fprintf(stderr, "ft-series: %s\n", what.c_str());
    std::exit(1);
}

Options parse(int argc, char** argv)
{
    Options o;
    for (int i = 1; i < argc; ++i) {
        const std::string key = argv[i];
        auto value = [&]() -> std::string {
            if (i + 1 >= argc) fail("missing value for " + key);
            return argv[++i];
        };
        if (key == "--implementation") o.implementation = value();
        else if (key == "--config") o.config = value();
        else if (key == "--out") o.out = value();
        else if (key == "--motion") o.motion = value();
        else if (key == "--input") o.input = value();
        else if (key == "--duration") o.duration = std::stod(value());
        else if (key == "--rate") o.rate = std::stod(value());
        else if (key == "--t0") o.t0 = std::stod(value());
        else if (key == "--seed") o.seed = std::stoull(value());
        else if (key == "--other-seed") { o.other_seed = std::stoull(value()); o.other = true; }
        else if (key == "--reset-at") o.reset_at = std::stod(value());
        else if (key == "--tare-at") o.tare_at = std::stod(value());
        else if (key == "--every") o.every = std::stoull(value());
        else fail("unknown option " + key);
    }
    if (o.config.empty() || o.out.empty()) fail("--config and --out are required");
    return o;
}

std::unique_ptr<sa::IForceTorqueSensorAdapter> make(const Options& o, std::uint64_t seed, const char* name)
{
    sa::ForceTorqueAdapterSetup setup;
    setup.physics_step_ns = kStepNs;
    setup.vehicle_name = "arm";
    setup.sensor_name = name;
    setup.has_random_seed = true;
    setup.random_seed = seed;
    setup.config_path = o.config;
    std::string error;
    std::unique_ptr<sa::IForceTorqueSensorAdapter> a;
    if (o.implementation == "hakoniwa") a = hs::create_hakoniwa_force_torque_sensor_adapter(setup, error);
    else if (o.implementation == "gazebo") a = sa::gazebo::create_gz_force_torque_sensor_adapter(setup, error);
    else fail("unknown implementation " + o.implementation);
    if (!a) fail(error);
    return a;
}

// The rows of --input (motion "file"): time and the six truth values.
std::vector<std::vector<double>> g_rows;

void load_input(const Options& o)
{
    std::ifstream in(o.input);
    if (!in) fail("cannot open --input " + o.input);
    std::string line;
    std::getline(in, line);  // header
    while (std::getline(in, line)) {
        std::vector<double> row;
        std::stringstream ss(line);
        for (std::string cell; std::getline(ss, cell, ',') && row.size() < 7;) row.push_back(std::stod(cell));
        if (row.size() < 7) fail("--input: a row has fewer than 7 columns");
        g_rows.push_back(row);
    }
    if (g_rows.size() < 2) fail("--input: fewer than two rows");
    kStepNs = static_cast<std::uint64_t>(std::llround((g_rows[1][0] - g_rows[0][0]) * 1e9));
    if (kStepNs == 0) fail("--input: the first two rows have the same time");
}

sa::Wrench truth(const Options& o, double t, std::uint64_t k)
{
    sa::Wrench w;
    if (o.motion == "file") {
        const auto& r = g_rows[k];
        w.force = {r[1], r[2], r[3]};
        w.torque = {r[4], r[5], r[6]};
    } else if (o.motion == "sine") {
        w.force = {5 * std::sin(2 * kPi * 0.7 * t), -3 + 2 * std::sin(2 * kPi * 1.3 * t),
                   -19.6 + 4 * std::sin(2 * kPi * 2.1 * t)};
        w.torque = {0.4 * std::sin(2 * kPi * 0.9 * t), 1.96 + 0.3 * std::sin(2 * kPi * 1.7 * t),
                    -0.2 * std::sin(2 * kPi * 3.1 * t)};
    } else if (o.motion == "ramp") {
        w.force.z = t >= o.t0 ? o.rate * (t - o.t0) : 0.0;
    } else if (o.motion == "step") {
        w.force.z = t >= o.t0 ? o.rate : 0.0;
    } else if (o.motion != "rest") {
        fail("unknown motion " + o.motion);
    }
    return w;
}

void put(std::ofstream& csv, const sa::Wrench& w)
{
    csv << ',' << w.force.x << ',' << w.force.y << ',' << w.force.z << ',' << w.torque.x << ',' << w.torque.y << ','
        << w.torque.z;
}

}  // namespace

int main(int argc, char** argv)
{
    const Options o = parse(argc, argv);
    if (o.motion == "file") {
        if (o.input.empty()) fail("--motion file needs --input");
        load_input(o);
    }
    auto sensor = make(o, o.seed, "ft");
    std::unique_ptr<sa::IForceTorqueSensorAdapter> other;
    if (o.other) other = make(o, o.other_seed, "ft_other");
    std::ofstream csv(o.out);
    csv.precision(17);  // enough digits to keep differences of rounding size
    csv << "time,updated,fx,fy,fz,tx,ty,tz,true_fx,true_fy,true_fz,true_tx,true_ty,true_tz,flags\n";
    const double steps_per_s = 1e9 / static_cast<double>(kStepNs);
    const auto steps = o.motion == "file" ? static_cast<std::uint64_t>(g_rows.size())
                                          : static_cast<std::uint64_t>(std::llround(o.duration * steps_per_s));
    const auto reset_step = o.reset_at >= 0 ? static_cast<std::uint64_t>(std::llround(o.reset_at * steps_per_s)) : steps + 1;
    const auto tare_step = o.tare_at >= 0 ? static_cast<std::uint64_t>(std::llround(o.tare_at * steps_per_s)) : steps + 1;
    std::string error;
    std::uint64_t k0 = 0;  // the step at which the sensor's own time restarted
    for (std::uint64_t k = 0; k < steps; ++k) {
        if (k == reset_step) {
            if (!sensor->reset(error)) fail(error);
            k0 = k;
        }
        if (k == tare_step && !sensor->tare(error)) fail("tare: " + error);
        const double t = static_cast<double>(k) * static_cast<double>(kStepNs) * 1e-9;
        sa::ForceTorqueTruth in;
        in.sim_time_ns = (k - k0) * kStepNs;
        in.wrench = truth(o, t, k);
        if (other) {
            sa::ForceTorqueSample ignored;
            if (!other->step(in, ignored, error)) fail(error);
        }
        sa::ForceTorqueSample s;
        if (!sensor->step(in, s, error)) fail(error);
        if (k % o.every != 0) continue;
        csv << t << ',' << (s.updated ? 1 : 0);
        put(csv, s.wrench);
        put(csv, in.wrench);
        csv << ',' << s.flags << '\n';
    }
    return 0;
}
