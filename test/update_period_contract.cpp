// The optional update-period calls (docs/spec/update-period.md): the interface's defaults,
// and an adapter that implements them.
#include "hakoniwa/sensor_adapter/imu_sensor_adapter.hpp"
#include "hakoniwa/sensor_adapter/baro_sensor_adapter.hpp"

#include <cstdio>
#include <string>
#include <vector>

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

// An IMU that leaves every period call to the interface.
class SilentImu final : public sa::IImuSensorAdapter {
public:
    bool reset(std::string&) override { return true; }
    const std::vector<sa::LogColumn>& log_columns() const override { return columns_; }
    void log_values(std::vector<double>& values) const override { values.clear(); }
    bool step(const sa::ImuTruth&, sa::ImuSample&, std::string&) override { return true; }
    sa::ImuAdapterCapabilities capabilities() const override { return {}; }
    bool calibration(sa::ImuCalibration&) const override { return false; }

private:
    std::vector<sa::LogColumn> columns_;
};

// A barometer with one period that can be read and changed (1 ms physics step).
class PeriodBaro final : public sa::IBaroSensorAdapter {
public:
    bool reset(std::string&) override
    {
        period_ns_ = 20000000;
        return true;
    }
    const std::vector<sa::LogColumn>& log_columns() const override { return columns_; }
    void log_values(std::vector<double>& values) const override { values.clear(); }
    bool step(const sa::BaroTruth&, sa::BaroSample&, std::string&) override { return true; }
    sa::BaroAdapterCapabilities capabilities() const override { return {}; }
    bool update_period_ns(std::uint64_t& period_ns) const override
    {
        period_ns = period_ns_;
        return true;
    }
    bool set_update_period_ns(std::uint64_t period_ns, std::string& error) override
    {
        if (period_ns == 0 || period_ns % 1000000 != 0) {
            error = "not a whole number of 1 ms steps";
            return false;
        }
        period_ns_ = period_ns;
        return true;
    }

private:
    std::vector<sa::LogColumn> columns_;
    std::uint64_t period_ns_{20000000};
};

// An IMU with one period for both parts: the default part calls follow it.
class OnePeriodImu final : public sa::IImuSensorAdapter {
public:
    bool reset(std::string&) override { return true; }
    const std::vector<sa::LogColumn>& log_columns() const override { return columns_; }
    void log_values(std::vector<double>& values) const override { values.clear(); }
    bool step(const sa::ImuTruth&, sa::ImuSample&, std::string&) override { return true; }
    sa::ImuAdapterCapabilities capabilities() const override { return {}; }
    bool calibration(sa::ImuCalibration&) const override { return false; }
    bool update_period_ns(std::uint64_t& period_ns) const override
    {
        period_ns = 1000000;
        return true;
    }

private:
    std::vector<sa::LogColumn> columns_;
};

}  // namespace

int main()
{
    std::string error;
    std::uint64_t period = 7;

    // [PER-010] [PER-120] the defaults: nothing to say, nothing changes
    SilentImu silent;
    check(!silent.update_period_ns(period) && period == 7, "default: no period, the argument unchanged");
    check(!silent.part_update_period_ns(sa::ImuPart::gyro, period) && period == 7, "default: no part period");
    check(!silent.set_update_period_ns(1000000, error) && !error.empty(), "default: a change is refused with a reason");
    error.clear();
    check(!silent.set_part_update_period_ns(sa::ImuPart::accel, 1000000, error) && !error.empty(),
          "default: a part change is refused with a reason");

    // [PER-010] [PER-110] [PER-120] [PER-150] an adapter that implements them
    PeriodBaro baro;
    check(baro.update_period_ns(period) && period == 20000000, "reads its period");
    check(baro.set_update_period_ns(10000000, error) && baro.update_period_ns(period) && period == 10000000,
          "changes its period");
    error.clear();
    check(!baro.set_update_period_ns(1500000, error) && !error.empty(), "refuses a period that is not whole steps");
    check(baro.update_period_ns(period) && period == 10000000, "a refused change leaves the period");
    check(baro.reset(error) && baro.update_period_ns(period) && period == 20000000, "reset returns the configured period");

    // [PER-020] [PER-130] one period for both IMU parts
    OnePeriodImu one;
    check(one.part_update_period_ns(sa::ImuPart::accel, period) && period == 1000000, "a part reads the IMU's period");
    error.clear();
    check(one.set_part_update_period_ns(sa::ImuPart::gyro, 1000000, error), "the period it has is accepted");
    check(!one.set_part_update_period_ns(sa::ImuPart::gyro, 2000000, error) && !error.empty(),
          "another period is refused");

    if (failures != 0) {
        std::fprintf(stderr, "%d failure(s)\n", failures);
        return 1;
    }
    std::printf("update_period_contract: all checks passed\n");
    return 0;
}
