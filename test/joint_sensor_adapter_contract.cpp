// Contract test for IJointSensorAdapter: an ideal adapter (no errors) with a sampling
// period shows how a host drives the interface for one joint.
#include "hakoniwa/sensor_adapter/joint_sensor_adapter.hpp"

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

// Ideal joint sensor: passes position, velocity and the transmitted torque through.
class IdealJointSensorAdapter : public sa::IJointSensorAdapter {
public:
    IdealJointSensorAdapter(const sa::JointSensorAdapterSetup& setup, std::uint64_t period_ns)
        : step_ns_(setup.physics_step_ns), period_ns_(period_ns)
    {
    }

    bool step(const sa::JointSensorTruth& truth, sa::JointSensorSample& out, std::string& error) override
    {
        if (truth.sim_time_ns != next_time_ns_) {
            error = "sim_time_ns must advance by physics_step_ns from 0";
            return false;
        }
        next_time_ns_ += step_ns_;
        if (truth.sim_time_ns % period_ns_ == 0) {
            last_ = {true, truth.sim_time_ns, truth.position, truth.velocity, truth.transmitted_torque, true, 0, 0};
        } else {
            last_.updated = false;
        }
        out = last_;
        return true;
    }

    bool reset(std::string&) override
    {
        next_time_ns_ = 0;
        last_ = {};
        return true;
    }

    sa::JointSensorAdapterCapabilities capabilities() const override
    {
        sa::JointSensorAdapterCapabilities caps{};
        caps.torque_sensor = true;
        caps.deterministic_with_seed = true;
        return caps;
    }

    const std::vector<sa::LogColumn>& log_columns() const override { return columns_; }
    void log_values(std::vector<double>& values) const override { values.clear(); }

private:
    std::uint64_t step_ns_;
    std::uint64_t period_ns_;
    std::uint64_t next_time_ns_{0};
    sa::JointSensorSample last_{};
    std::vector<sa::LogColumn> columns_;
};

}  // namespace

int main()
{
    sa::JointSensorAdapterSetup setup;
    setup.physics_step_ns = 1000000;  // 1 ms
    setup.sensor_name = "shoulder_pan";
    IdealJointSensorAdapter adapter(setup, 10000000);  // 100 Hz
    std::string error;
    sa::JointSensorSample out;
    for (std::uint64_t k = 0; k < 25; ++k) {
        sa::JointSensorTruth truth;
        truth.sim_time_ns = k * setup.physics_step_ns;
        truth.position = 0.01 * static_cast<double>(k);
        truth.velocity = 10.0;
        truth.actuator_torque = 1.5;
        truth.transmitted_torque = 1.5;
        check(adapter.step(truth, out, error), "step");
        check(out.updated == (k % 10 == 0), "measures every 10 ms");
        const double measured = 0.01 * static_cast<double>(k - k % 10);
        check(std::fabs(out.position - measured) < 1e-12, "position held between samples");
        check(out.has_effort && out.effort == 1.5, "effort from the transmitted torque");
    }
    sa::JointSensorTruth late;
    late.sim_time_ns = 40 * setup.physics_step_ns;
    check(!adapter.step(late, out, error), "skipped time is an error");
    check(adapter.reset(error), "reset");
    late.sim_time_ns = 0;
    check(adapter.step(late, out, error) && out.updated, "step from 0 after reset");
    if (failures == 0) std::printf("joint_sensor_adapter_contract: OK\n");
    return failures == 0 ? EXIT_SUCCESS : EXIT_FAILURE;
}
