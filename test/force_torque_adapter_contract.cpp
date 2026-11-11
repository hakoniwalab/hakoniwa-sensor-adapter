// Contract test for IForceTorqueSensorAdapter: an ideal adapter (no errors) with a
// sampling period and tare shows how a host drives the interface.
#include "hakoniwa/sensor_adapter/force_torque_sensor_adapter.hpp"

#include <cmath>
#include <cstdio>
#include <cstdlib>

namespace sa = hakoniwa::sensor_adapter;

namespace {

int failures = 0;

void check(bool condition, const char* what)
{
    if (!condition) {
        std::fprintf(stderr, "FAIL: %s\n", what);
        ++failures;
    }
}

bool near(double a, double b) { return std::fabs(a - b) < 1e-12; }

// Ideal force-torque sensor: passes the truth through, measuring every period_ns.
// tare() takes the last measured value (an ideal sensor has no noise to average).
class IdealForceTorqueAdapter : public sa::IForceTorqueSensorAdapter {
public:
    IdealForceTorqueAdapter(const sa::ForceTorqueAdapterSetup& setup, std::uint64_t period_ns)
        : step_ns_(setup.physics_step_ns), period_ns_(period_ns)
    {
        columns_ = {{"tare_fz", "N"}};
    }

    bool step(const sa::ForceTorqueTruth& truth, sa::ForceTorqueSample& out, std::string& error) override
    {
        if (truth.sim_time_ns != next_time_ns_) {
            error = "sim_time_ns must advance by physics_step_ns from 0";
            return false;
        }
        next_time_ns_ += step_ns_;
        if (truth.sim_time_ns % period_ns_ == 0) {
            raw_ = truth.wrench;
            measured_ = true;
            last_.updated = true;
            last_.sample_time_ns = truth.sim_time_ns;
            last_.wrench = subtract(raw_, tare_);
            last_.flags = 0;
        } else {
            last_.updated = false;
        }
        out = last_;
        return true;
    }

    bool tare(std::string& error) override
    {
        if (!measured_) {
            error = "no sample measured yet";
            return false;
        }
        tare_ = raw_;
        return true;
    }

    void clear_tare() override { tare_ = {}; }

    bool reset(std::string&) override
    {
        next_time_ns_ = 0;
        measured_ = false;
        raw_ = {};
        tare_ = {};
        last_ = {};
        return true;
    }

    sa::ForceTorqueAdapterCapabilities capabilities() const override
    {
        sa::ForceTorqueAdapterCapabilities caps{};
        caps.tare = true;
        caps.deterministic_with_seed = true;
        return caps;
    }

    const std::vector<sa::LogColumn>& log_columns() const override { return columns_; }

    void log_values(std::vector<double>& values) const override { values.assign(1, tare_.force.z); }

private:
    static sa::Wrench subtract(const sa::Wrench& a, const sa::Wrench& b)
    {
        sa::Wrench w;
        w.force = {a.force.x - b.force.x, a.force.y - b.force.y, a.force.z - b.force.z};
        w.torque = {a.torque.x - b.torque.x, a.torque.y - b.torque.y, a.torque.z - b.torque.z};
        return w;
    }

    std::uint64_t step_ns_;
    std::uint64_t period_ns_;
    std::uint64_t next_time_ns_{0};
    bool measured_{false};
    sa::Wrench raw_{};
    sa::Wrench tare_{};
    sa::ForceTorqueSample last_{};
    std::vector<sa::LogColumn> columns_;
};

}  // namespace

int main()
{
    sa::ForceTorqueAdapterSetup setup;
    setup.physics_step_ns = 1000000;  // 1 ms
    check(setup.sensor_name == "force_torque", "default sensor name");

    IdealForceTorqueAdapter adapter(setup, 2000000);  // 500 Hz
    std::string error;
    sa::ForceTorqueSample out;

    check(!adapter.tare(error), "tare before the first sample fails");

    // A 2 kg tool hanging at rest, sensor z up: the tool pulls the sensor down.
    sa::ForceTorqueTruth truth;
    truth.wrench.force = {0.0, 0.0, -2.0 * 9.80665};
    for (std::uint64_t k = 0; k < 4; ++k) {
        truth.sim_time_ns = k * setup.physics_step_ns;
        check(adapter.step(truth, out, error), "step");
        check(out.updated == (k % 2 == 0), "measures on the 2 ms boundaries only");
        check(near(out.wrench.force.z, -2.0 * 9.80665), "tool weight is part of the measurement");
    }

    // Tare removes the tool weight; a contact force then reads alone.
    check(adapter.tare(error), "tare after a sample");
    truth.sim_time_ns = 4 * setup.physics_step_ns;
    truth.wrench.force.z = -2.0 * 9.80665 - 5.0;  // plus 5 N pushing down
    check(adapter.step(truth, out, error), "step after tare");
    check(near(out.wrench.force.z, -5.0), "tared value is the contact force");

    adapter.clear_tare();
    truth.sim_time_ns = 5 * setup.physics_step_ns;
    check(adapter.step(truth, out, error), "step after clear_tare (held)");
    check(!out.updated, "odd step holds the value");

    // Time that does not advance by the physics step is an error.
    truth.sim_time_ns = 9 * setup.physics_step_ns;
    check(!adapter.step(truth, out, error), "skipped time is an error");

    check(adapter.reset(error), "reset");
    check(!adapter.tare(error), "tare after reset fails until a sample");
    truth.sim_time_ns = 0;
    check(adapter.step(truth, out, error), "step from 0 after reset");
    check(out.flags == 0, "no flags for an ideal sensor");
    check(static_cast<std::uint32_t>(sa::kForceTorqueFlagSaturated) == static_cast<std::uint32_t>(sa::kSensorFlagSaturated), "saturated is the shared bit");

    if (failures == 0) std::printf("force_torque_adapter_contract: OK\n");
    return failures == 0 ? EXIT_SUCCESS : EXIT_FAILURE;
}
