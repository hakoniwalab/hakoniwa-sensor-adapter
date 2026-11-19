// Force-torque comparison: the Hakoniwa adapter (hakoniwa-sensor-pro, by name) and the gz-sensors adapter
// through the public interface, with the same truth. Axes 0 (no-noise agreement), 1 (accuracy), 3 (repeatability)
// and 4 (speed) of the sensor evaluation. Prints the tables and writes CSV to OUT_DIR.
//
// Usage: compare_force_torque OUT_DIR
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "hakoniwa_entry.hpp"
#include "hakoniwa/sensor_adapter/gazebo/gz_force_torque_sensor_adapter.hpp"

namespace sa = hakoniwa::sensor_adapter;
namespace fs = std::filesystem;
using Adapter = std::unique_ptr<sa::IForceTorqueSensorAdapter>;

namespace {

constexpr std::uint64_t kStepNs = 1000000;  // 1 ms
constexpr double kPi = 3.14159265358979323846;
fs::path out_dir;

[[noreturn]] void fail(const std::string& what)
{
    std::fprintf(stderr, "error: %s\n", what.c_str());
    std::exit(1);
}

std::string config(const std::string& file) { return std::string(HAKO_FT_COMPARE_CONFIG_DIR) + "/" + file; }

Adapter make(const std::string& impl, const std::string& name, std::uint64_t seed, const char* sensor = "ft")
{
    sa::ForceTorqueAdapterSetup setup;
    setup.physics_step_ns = kStepNs;
    setup.vehicle_name = "arm";
    setup.sensor_name = sensor;
    setup.has_random_seed = true;
    setup.random_seed = seed;
    std::string error;
    Adapter a;
    if (impl == "hakoniwa") {
        setup.config_path = config("hk-" + name + ".json");
        a = hs::create_hakoniwa_force_torque_sensor_adapter(setup, error);
    } else {
        setup.config_path = config("gz-" + name + ".sdf");
        a = hakoniwa::sensor_adapter::gazebo::create_gz_force_torque_sensor_adapter(setup, error);
    }
    if (!a) fail(impl + " " + name + ": " + error);
    return a;
}

double axis(const sa::Wrench& w, int i)
{
    const double v[6] = {w.force.x, w.force.y, w.force.z, w.torque.x, w.torque.y, w.torque.z};
    return v[i];
}

// Truth: each axis a sine of its own amplitude and frequency, plus a constant.
sa::Wrench moving_truth(std::uint64_t k)
{
    const double t = static_cast<double>(k) * 1e-3;
    sa::Wrench w;
    w.force = {5 * std::sin(2 * kPi * 0.7 * t), -3 + 2 * std::sin(2 * kPi * 1.3 * t), -19.6 + 4 * std::sin(2 * kPi * 2.1 * t)};
    w.torque = {0.4 * std::sin(2 * kPi * 0.9 * t), 1.96 + 0.3 * std::sin(2 * kPi * 1.7 * t), -0.2 * std::sin(2 * kPi * 3.1 * t)};
    return w;
}

sa::Wrench zero_truth(std::uint64_t) { return {}; }

// Steps n times and returns the updated samples.
std::vector<sa::ForceTorqueSample> run(sa::IForceTorqueSensorAdapter& a, std::uint64_t n,
                                       const std::function<sa::Wrench(std::uint64_t)>& truth,
                                       std::uint64_t first = 0)
{
    std::vector<sa::ForceTorqueSample> out;
    std::string error;
    for (std::uint64_t k = first; k < first + n; ++k) {
        sa::ForceTorqueTruth t;
        t.sim_time_ns = k * kStepNs;
        t.wrench = truth(k);
        sa::ForceTorqueSample s;
        if (!a.step(t, s, error)) fail(error);
        if (s.updated) out.push_back(s);
    }
    return out;
}

double mean(const std::vector<double>& v)
{
    double m = 0;
    for (double x : v) m += x;
    return m / static_cast<double>(v.size());
}

double stddev(const std::vector<double>& v)
{
    const double m = mean(v);
    double s = 0;
    for (double x : v) s += (x - m) * (x - m);
    return std::sqrt(s / static_cast<double>(v.size() - 1));
}

std::vector<double> column(const std::vector<sa::ForceTorqueSample>& s, int i)
{
    std::vector<double> v;
    for (const auto& x : s) v.push_back(axis(x.wrench, i));
    return v;
}

const char* kAxis[6] = {"Fx", "Fy", "Fz", "Tx", "Ty", "Tz"};
const char* kImpl[2] = {"hakoniwa", "gazebo"};

// Axis 0: errors off, the same truth; values and sample times compared one by one.
void axis0()
{
    std::printf("\n## Axis 0: no-noise agreement (10 s, 500 Hz, moving truth)\n\n");
    auto hk = make("hakoniwa", "ideal", 1);
    auto gz = make("gazebo", "ideal", 1);
    const auto a = run(*hk, 10000, moving_truth);
    const auto b = run(*gz, 10000, moving_truth);
    if (a.size() != b.size()) fail("axis0: sample counts differ");
    std::ofstream csv(out_dir / "axis0.csv");
    csv << "axis,max_abs_diff_hakoniwa_gazebo,max_abs_diff_hakoniwa_truth\n";
    std::size_t time_mismatch = 0;
    for (std::size_t k = 0; k < a.size(); ++k) time_mismatch += a[k].sample_time_ns != b[k].sample_time_ns;
    std::printf("| axis | max |hakoniwa - gazebo| | max |hakoniwa - truth| |\n|---|---|---|\n");
    for (int i = 0; i < 6; ++i) {
        double d = 0, dt = 0;
        for (std::size_t k = 0; k < a.size(); ++k) {
            d = std::max(d, std::fabs(axis(a[k].wrench, i) - axis(b[k].wrench, i)));
            dt = std::max(dt, std::fabs(axis(a[k].wrench, i) - axis(moving_truth(a[k].sample_time_ns / kStepNs), i)));
        }
        std::printf("| %s | %.3g | %.3g |\n", kAxis[i], d, dt);
        csv << kAxis[i] << "," << d << "," << dt << "\n";
    }
    std::printf("\nsamples %zu each, sample-time mismatches %zu\n", a.size(), time_mismatch);
}

// Axis 1: the same meaning of error settings, statistics against the configuration.
void axis1()
{
    std::printf("\n## Axis 1: accuracy\n\n");
    std::ofstream csv(out_dir / "axis1.csv");
    csv << "item,impl,axis,configured,measured\n";
    const double sigma[6] = {0.1, 0.1, 0.2, 0.01, 0.01, 0.02};
    const double step[6] = {0.05, 0.05, 0.05, 0.005, 0.005, 0.005};

    std::printf("### Noise sigma (20,000 samples)\n\n| axis | configured | hakoniwa | gazebo |\n|---|---|---|---|\n");
    std::vector<std::vector<double>> measured(2);
    for (int m = 0; m < 2; ++m) {
        auto a = make(kImpl[m], "noise", 11);
        const auto s = run(*a, 40000, zero_truth);
        for (int i = 0; i < 6; ++i) measured[m].push_back(stddev(column(s, i)));
    }
    for (int i = 0; i < 6; ++i) {
        std::printf("| %s | %.4g | %.4g | %.4g |\n", kAxis[i], sigma[i], measured[0][i], measured[1][i]);
        for (int m = 0; m < 2; ++m) csv << "noise_sigma," << kImpl[m] << "," << kAxis[i] << "," << sigma[i] << "," << measured[m][i] << "\n";
    }

    std::printf("\n### Quantization (values that are not a multiple of the step; distinct values)\n\n| axis | step | hakoniwa off-grid | gazebo off-grid |\n|---|---|---|---|\n");
    for (int i = 0; i < 6; ++i) {
        std::size_t off[2] = {0, 0};
        for (int m = 0; m < 2; ++m) {
            auto a = make(kImpl[m], "quant", 12);
            for (double v : column(run(*a, 20000, zero_truth), i)) {
                const double r = v / step[i];
                if (std::fabs(r - std::round(r)) > 1e-6) ++off[m];
            }
            csv << "quant_offgrid," << kImpl[m] << "," << kAxis[i] << "," << step[i] << "," << off[m] << "\n";
        }
        std::printf("| %s | %g | %zu / 10000 | %zu / 10000 |\n", kAxis[i], step[i], off[0], off[1]);
    }

    std::printf("\n### Bias over 40 units (seeds 1..40): Fx, configured 0.5 N\n\n| impl | min | max | negative units | distinct values |\n|---|---|---|---|---|\n");
    for (int m = 0; m < 2; ++m) {
        std::vector<double> b;
        for (std::uint64_t seed = 1; seed <= 40; ++seed) {
            auto a = make(kImpl[m], "bias", seed);
            b.push_back(axis(run(*a, 2, zero_truth)[0].wrench, 0));
        }
        auto sorted = b;
        std::sort(sorted.begin(), sorted.end());
        const auto distinct = std::unique(sorted.begin(), sorted.end(), [](double x, double y) { return std::fabs(x - y) < 1e-12; }) - sorted.begin();
        const auto negative = std::count_if(b.begin(), b.end(), [](double x) { return x < 0; });
        std::printf("| %s | %.4g | %.4g | %td | %td |\n", kImpl[m], sorted.front(), sorted.back(), negative, distinct);
        csv << "bias_min," << kImpl[m] << ",Fx,0.5," << sorted.front() << "\nbias_max," << kImpl[m] << ",Fx,0.5," << sorted.back() << "\n";
    }

    std::printf("\n### Drift (each against its own model)\n\n");
    {
        auto a = make("hakoniwa", "drift", 13);
        const auto s = run(*a, 400000, zero_truth);  // 400 s
        std::vector<double> inc;
        for (std::size_t k = 1; k < s.size(); ++k) inc.push_back(s[k].wrench.force.x - s[k - 1].wrench.force.x);
        const double expected = 60.0 * std::sqrt(0.002 / 3600.0);
        std::printf("- hakoniwa random walk, 60 N per hour: increment sigma %.4g (expected %.4g)\n", stddev(inc), expected);
        csv << "drift_increment_sigma,hakoniwa,Fx," << expected << "," << stddev(inc) << "\n";
    }
    {
        // Stationary sigma of the Gauss-Markov dynamic bias over many independent units.
        std::vector<double> end;
        for (std::uint64_t seed = 1; seed <= 200; ++seed) {
            auto a = make("gazebo", "drift", seed);
            const auto s = run(*a, 60000, zero_truth);  // 60 s = 6 correlation times
            end.push_back(s.back().wrench.force.x);
        }
        // gz-sensors' discrete step gives a stationary sigma of dynamic_bias_stddev * sqrt(tau / 2).
        const double expected = 1.0 * std::sqrt(10.0 / 2.0);
        std::printf("- gazebo Gauss-Markov, dynamic_bias_stddev 1 N, tau 10 s: stationary sigma over 200 units %.4g "
                    "(gz-sensors' formula sigma_b * sqrt(tau / 2) = %.4g)\n", stddev(end), expected);
        csv << "drift_stationary_sigma,gazebo,Fx," << expected << "," << stddev(end) << "\n";
    }
}

// Axis 3: repeatability in five situations.
void axis3()
{
    std::printf("\n## Axis 3: repeatability (noise config, 2,000 samples, seed 5)\n\n| situation | hakoniwa | gazebo |\n|---|---|---|\n");
    std::ofstream csv(out_dir / "axis3.csv");
    csv << "situation,hakoniwa_same,gazebo_same\n";
    auto same = [](const std::vector<sa::ForceTorqueSample>& x, const std::vector<sa::ForceTorqueSample>& y) {
        if (x.size() != y.size()) return false;
        for (std::size_t k = 0; k < x.size(); ++k)
            for (int i = 0; i < 6; ++i)
                if (axis(x[k].wrench, i) != axis(y[k].wrench, i)) return false;
        return true;
    };
    struct Row {
        const char* name;
        bool result[2];
    };
    std::vector<Row> rows = {{"created again with the same seed", {}}, {"another sensor stepped in between", {}},
                             {"two sensors with the same seed, stepped alternately", {}}, {"after reset", {}},
                             {"a different seed gives a different sequence", {}}};
    for (int m = 0; m < 2; ++m) {
        auto base = make(kImpl[m], "noise", 5);
        const auto ref = run(*base, 4000, zero_truth);
        {
            auto again = make(kImpl[m], "noise", 5);
            rows[0].result[m] = same(ref, run(*again, 4000, zero_truth));
        }
        {
            auto a = make(kImpl[m], "noise", 5, "ft_a");
            auto other = make(kImpl[m], "noise", 99, "ft_other");
            std::vector<sa::ForceTorqueSample> got;
            for (std::uint64_t k = 0; k < 4000; ++k) {
                run(*other, 1, zero_truth, k);
                const auto s = run(*a, 1, zero_truth, k);
                got.insert(got.end(), s.begin(), s.end());
            }
            auto a_alone = make(kImpl[m], "noise", 5, "ft_a");
            rows[1].result[m] = same(run(*a_alone, 4000, zero_truth), got);
        }
        {
            auto a = make(kImpl[m], "noise", 5, "ft_a");
            auto b = make(kImpl[m], "noise", 5, "ft_a");
            std::vector<sa::ForceTorqueSample> ga, gb;
            for (std::uint64_t k = 0; k < 4000; ++k) {
                const auto x = run(*a, 1, zero_truth, k);
                const auto y = run(*b, 1, zero_truth, k);
                ga.insert(ga.end(), x.begin(), x.end());
                gb.insert(gb.end(), y.begin(), y.end());
            }
            rows[2].result[m] = same(ga, gb);
        }
        {
            std::string error;
            if (!base->reset(error)) fail(error);
            rows[3].result[m] = same(ref, run(*base, 4000, zero_truth));
        }
        {
            auto other = make(kImpl[m], "noise", 6);
            rows[4].result[m] = !same(ref, run(*other, 4000, zero_truth));
        }
    }
    for (const auto& r : rows) {
        std::printf("| %s | %s | %s |\n", r.name, r.result[0] ? "yes" : "no", r.result[1] ? "yes" : "no");
        csv << r.name << "," << r.result[0] << "," << r.result[1] << "\n";
    }
}

// Axis 4: time per physics step.
void axis4()
{
    std::printf("\n## Axis 4: time per physics step (60 s at 1 ms, median of 5 runs)\n\n| config | hakoniwa [us] | gazebo [us] |\n|---|---|---|\n");
    std::ofstream csv(out_dir / "axis4.csv");
    csv << "config,hakoniwa_us,gazebo_us\n";
    for (const char* name : {"ideal", "noise"}) {
        double us[2];
        for (int m = 0; m < 2; ++m) {
            std::vector<double> runs;
            for (int r = 0; r < 5; ++r) {
                auto a = make(kImpl[m], name, 7);
                const auto t0 = std::chrono::steady_clock::now();
                run(*a, 60000, moving_truth);
                const auto t1 = std::chrono::steady_clock::now();
                runs.push_back(std::chrono::duration<double, std::micro>(t1 - t0).count() / 60000.0);
            }
            std::sort(runs.begin(), runs.end());
            us[m] = runs[2];
        }
        std::printf("| %s | %.3f | %.3f |\n", name, us[0], us[1]);
        csv << name << "," << us[0] << "," << us[1] << "\n";
    }
}

}  // namespace

int main(int argc, char** argv)
{
    if (argc != 2) fail("usage: compare_force_torque OUT_DIR");
    out_dir = argv[1];
    fs::create_directories(out_dir);
    axis0();
    axis1();
    axis3();
    axis4();
    return 0;
}
