// Contract test for IImuSensorAdapter: an ideal adapter (no errors) with a sampling
// period shows how a host drives the interface.
#include "hakoniwa/sensor_adapter/imu_sensor_adapter.hpp"

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

// Ideal IMU: passes the truth through, measuring every period_ns.
class IdealImuAdapter : public sa::IImuSensorAdapter {
public:
    IdealImuAdapter(const sa::ImuAdapterSetup& setup, std::uint64_t period_ns)
        : step_ns_(setup.physics_step_ns), period_ns_(period_ns)
    {
        columns_ = {{"samples", "count"}};
    }

    bool step(const sa::ImuTruth& truth, sa::ImuSample& out, std::string& error) override
    {
        if (truth.sim_time_ns != next_time_ns_) {
            error = "sim_time_ns must advance by physics_step_ns from 0";
            return false;
        }
        next_time_ns_ += step_ns_;
        const bool measure = truth.sim_time_ns % period_ns_ == 0;
        if (measure) {
            last_.gyro = {true, truth.sim_time_ns, truth.angular_velocity, 0};
            last_.accel = {true, truth.sim_time_ns, truth.specific_force, 0};
            ++samples_;
        } else {
            last_.gyro.updated = false;
            last_.accel.updated = false;
        }
        out = last_;
        return true;
    }

    bool reset(std::string&) override
    {
        next_time_ns_ = 0;
        samples_ = 0;
        last_ = {};
        return true;
    }

    sa::ImuAdapterCapabilities capabilities() const override
    {
        sa::ImuAdapterCapabilities caps{};
        caps.deterministic_with_seed = true;
        return caps;
    }

    bool calibration(sa::ImuCalibration&) const override { return false; }

    const std::vector<sa::LogColumn>& log_columns() const override { return columns_; }

    void log_values(std::vector<double>& values) const override
    {
        values.assign(1, static_cast<double>(samples_));
    }

private:
    std::uint64_t step_ns_;
    std::uint64_t period_ns_;
    std::uint64_t next_time_ns_{0};
    std::uint64_t samples_{0};
    sa::ImuSample last_{};
    std::vector<sa::LogColumn> columns_;
};

}  // namespace

int main()
{
    sa::ImuAdapterSetup setup{};
    setup.physics_step_ns = 1000000;  // 1 ms
    IdealImuAdapter adapter(setup, 4000000);  // measures every 4 ms
    sa::IImuSensorAdapter& imu = adapter;
    std::string error;

    // At rest and level, an ideal accelerometer reads -g along the sensor z (FRD).
    sa::ImuTruth truth{};
    truth.specific_force = {0.0, 0.0, -9.80665};

    sa::ImuSample sample{};
    for (int k = 0; k < 6; ++k) {
        truth.sim_time_ns = static_cast<std::uint64_t>(k) * setup.physics_step_ns;
        truth.angular_velocity = {0.1 * k, 0.0, 0.0};
        check(imu.step(truth, sample, error), "step succeeds");
        const bool boundary = (k % 4) == 0;
        check(sample.gyro.updated == boundary, "measures only on the period boundary");
        // Between boundaries the last measured value is held.
        const double expected = 0.1 * (k - k % 4);
        check(sample.gyro.value.x == expected, "value is held between measurements");
        check(sample.gyro.sample_time_ns == static_cast<std::uint64_t>(k - k % 4) * setup.physics_step_ns,
              "sample time is the measurement time");
        check(sample.accel.value.z == -9.80665, "specific force at rest");
        check(sample.gyro.flags == 0, "no undefined flag bits");
    }

    std::vector<double> values;
    check(imu.log_columns().size() == 1, "log columns are declared");
    imu.log_values(values);
    check(values.size() == imu.log_columns().size() && values[0] == 2.0, "log values follow the columns");

    truth.sim_time_ns = 99;  // does not advance by the physics step
    check(!imu.step(truth, sample, error) && !error.empty(), "a time jump is an error");

    check(imu.reset(error), "reset succeeds");
    truth.sim_time_ns = 0;
    check(imu.step(truth, sample, error) && sample.gyro.updated, "after reset, time starts at 0");

    sa::ImuCalibration calibration{};
    check(!imu.capabilities().calibration_values && !imu.calibration(calibration),
          "calibration follows the capability");

    if (failures == 0) {
        std::printf("imu_sensor_adapter_contract: OK\n");
    }
    return failures == 0 ? EXIT_SUCCESS : EXIT_FAILURE;
}
