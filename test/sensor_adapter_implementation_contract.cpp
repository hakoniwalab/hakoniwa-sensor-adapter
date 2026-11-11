// Contract test for SensorAdapterImplementation: a fake implementation that provides
// only the joint sensor shows how an implementation publishes its descriptor through
// the fixed entry point and how a host finds it from the implementation name.
#include "hakoniwa/sensor_adapter/sensor_adapter_implementation.hpp"

#include <cstdio>
#include <cstdlib>
#include <cstring>

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

class PassThroughJointSensorAdapter : public sa::IJointSensorAdapter {
public:
    bool reset(std::string&) override { return true; }
    const std::vector<sa::LogColumn>& log_columns() const override { return columns_; }
    void log_values(std::vector<double>& values) const override { values.clear(); }
    bool step(const sa::JointSensorTruth& truth, sa::JointSensorSample& out, std::string&) override
    {
        out.updated = true;
        out.sample_time_ns = truth.sim_time_ns;
        out.position = truth.position;
        out.velocity = truth.velocity;
        return true;
    }
    sa::JointSensorAdapterCapabilities capabilities() const override { return {}; }

private:
    std::vector<sa::LogColumn> columns_;
};

// The implementation side: its creation function and its descriptor.
std::unique_ptr<sa::IJointSensorAdapter> create_fake_joint(
    const sa::JointSensorAdapterSetup& setup, std::string& error)
{
    if (setup.config_path == "missing.json") {
        error = "cannot open missing.json";
        return nullptr;
    }
    return std::make_unique<PassThroughJointSensorAdapter>();
}

}  // namespace

HAKO_SENSOR_ADAPTER_DEFINE_IMPLEMENTATION(fake)
{
    static const sa::SensorAdapterImplementation implementation = [] {
        sa::SensorAdapterImplementation value;
        value.name = "fake";
        value.create_joint = &create_fake_joint;
        return value;
    }();
    return &implementation;
}

// The host side only needs the name.
HAKO_SENSOR_ADAPTER_DECLARE_IMPLEMENTATION(fake);

int main()
{
    const sa::SensorAdapterImplementation* implementation =
        HAKO_SENSOR_ADAPTER_IMPLEMENTATION_ENTRY(fake)();
    check(implementation != nullptr, "the entry point returns a descriptor");
    if (implementation == nullptr) {
        return EXIT_FAILURE;
    }
    check(implementation->version == sa::kSensorAdapterImplementationVersion, "version is the current one");
    check(implementation->name != nullptr && std::strcmp(implementation->name, "fake") == 0,
          "name matches the entry point name");
    check(implementation == HAKO_SENSOR_ADAPTER_IMPLEMENTATION_ENTRY(fake)(), "the descriptor is a single object");

    const auto types = sa::provided_sensor_types(*implementation);
    check(types.size() == 1 && types[0] == "joint", "only the joint sensor is provided");
    check(implementation->create_imu == nullptr && implementation->create_force_torque == nullptr
              && implementation->create_lidar == nullptr && implementation->create_rgb_camera == nullptr,
          "sensor types not provided are nullptr");

    sa::JointSensorAdapterSetup setup;
    setup.physics_step_ns = 1000000;
    std::string error;
    auto adapter = implementation->create_joint(setup, error);
    check(adapter != nullptr && error.empty(), "the joint adapter is created");
    if (adapter) {
        sa::JointSensorTruth truth;
        truth.position = 0.5;
        truth.velocity = -0.25;
        sa::JointSensorSample sample;
        check(adapter->step(truth, sample, error), "the created adapter steps");
        check(sample.updated && sample.position == 0.5 && sample.velocity == -0.25,
              "the created adapter is the implementation's adapter");
    }

    setup.config_path = "missing.json";
    error.clear();
    check(implementation->create_joint(setup, error) == nullptr && !error.empty(),
          "a creation failure returns nullptr and an error");

    sa::SensorAdapterImplementation all;
    check(sa::provided_sensor_types(all).empty(), "an empty descriptor provides nothing");

    if (failures != 0) {
        std::fprintf(stderr, "%d check(s) failed\n", failures);
        return EXIT_FAILURE;
    }
    std::printf("sensor_adapter_implementation_contract: OK\n");
    return EXIT_SUCCESS;
}
