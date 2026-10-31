// Contract test for the magnetometer, barometer and GPS adapter interfaces: ideal
// adapters (no errors, measured every period) driven through the interfaces.
#include "hakoniwa/sensor_adapter/baro_sensor_adapter.hpp"
#include "hakoniwa/sensor_adapter/gps_sensor_adapter.hpp"
#include "hakoniwa/sensor_adapter/mag_sensor_adapter.hpp"

#include <cstdio>
#include <cstdlib>
#include <memory>
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

// Shared timing of the ideal adapters: time 0, dt, 2 dt, ... and a measurement every period.
class IdealTiming {
public:
    IdealTiming(std::uint64_t step_ns, std::uint64_t period_ns) : step_ns_(step_ns), period_ns_(period_ns) {}

    // Returns -1 on a time error, 1 when this step measures, 0 otherwise.
    int advance(std::uint64_t sim_time_ns, std::string& error)
    {
        if (sim_time_ns != next_ns_) {
            error = "sim_time_ns must advance by physics_step_ns from 0";
            return -1;
        }
        next_ns_ += step_ns_;
        return sim_time_ns % period_ns_ == 0 ? 1 : 0;
    }
    void reset() { next_ns_ = 0; }

private:
    std::uint64_t step_ns_, period_ns_, next_ns_{0};
};

const std::vector<sa::LogColumn> kNoColumns;

class IdealMag : public sa::IMagSensorAdapter {
public:
    explicit IdealMag(const sa::MagAdapterSetup& s) : timing_(s.physics_step_ns, 20000000) {}
    bool step(const sa::MagTruth& truth, sa::MagSample& out, std::string& error) override
    {
        const int m = timing_.advance(truth.sim_time_ns, error);
        if (m < 0) return false;
        if (m == 1) last_ = {true, truth.sim_time_ns, truth.magnetic_field, 0};
        else last_.updated = false;
        out = last_;
        return true;
    }
    sa::MagAdapterCapabilities capabilities() const override { return {}; }
    bool reset(std::string&) override { timing_.reset(); last_ = {}; return true; }
    const std::vector<sa::LogColumn>& log_columns() const override { return kNoColumns; }
    void log_values(std::vector<double>& v) const override { v.clear(); }
private:
    IdealTiming timing_;
    sa::MagSample last_{};
};

class IdealBaro : public sa::IBaroSensorAdapter {
public:
    explicit IdealBaro(const sa::BaroAdapterSetup& s) : timing_(s.physics_step_ns, 10000000) {}
    bool step(const sa::BaroTruth& truth, sa::BaroSample& out, std::string& error) override
    {
        const int m = timing_.advance(truth.sim_time_ns, error);
        if (m < 0) return false;
        if (m == 1) {
            // Linear stand-in for an atmosphere model (about 12 Pa per metre near sea level).
            last_ = {true, truth.sim_time_ns, truth.sea_level_pressure_pa - 12.0 * truth.altitude_m, 0.0, 0.0, 0};
        } else {
            last_.updated = false;
        }
        out = last_;
        return true;
    }
    sa::BaroAdapterCapabilities capabilities() const override
    {
        sa::BaroAdapterCapabilities caps{};
        caps.sea_level_weather = true;
        return caps;
    }
    bool reset(std::string&) override { timing_.reset(); last_ = {}; return true; }
    const std::vector<sa::LogColumn>& log_columns() const override { return kNoColumns; }
    void log_values(std::vector<double>& v) const override { v.clear(); }
private:
    IdealTiming timing_;
    sa::BaroSample last_{};
};

class IdealGps : public sa::IGpsSensorAdapter {
public:
    explicit IdealGps(const sa::GpsAdapterSetup& s) : timing_(s.physics_step_ns, 100000000) {}
    bool step(const sa::GpsTruth& truth, sa::GpsSample& out, std::string& error) override
    {
        const int m = timing_.advance(truth.sim_time_ns, error);
        if (m < 0) return false;
        if (m == 1) {
            last_ = {};
            last_.updated = true;
            last_.sample_time_ns = truth.sim_time_ns;
            last_.position = truth.position;
            last_.velocity_nwu = truth.velocity_nwu;
            last_.fix_type = 3;
        } else {
            last_.updated = false;
        }
        out = last_;
        return true;
    }
    sa::GpsAdapterCapabilities capabilities() const override { return {}; }
    bool reset(std::string&) override { timing_.reset(); last_ = {}; return true; }
    const std::vector<sa::LogColumn>& log_columns() const override { return kNoColumns; }
    void log_values(std::vector<double>& v) const override { v.clear(); }
private:
    IdealTiming timing_;
    sa::GpsSample last_{};
};

}  // namespace

int main()
{
    std::string error;

    sa::MagAdapterSetup mag_setup{};
    sa::BaroAdapterSetup baro_setup{};
    sa::GpsAdapterSetup gps_setup{};
    gps_setup.origin = {47.641468, -122.140165, 121.321};
    check(mag_setup.sensor_name == "mag" && baro_setup.sensor_name == "baro" && gps_setup.sensor_name == "gps",
          "default sensor names");

    IdealMag mag(mag_setup);
    IdealBaro baro(baro_setup);
    IdealGps gps(gps_setup);

    // Every adapter is also an ISensorAdapter (reset and log getters).
    std::vector<sa::ISensorAdapter*> all{&mag, &baro, &gps};

    for (int k = 0; k < 120; ++k) {
        const std::uint64_t t = static_cast<std::uint64_t>(k) * 1000000;

        sa::MagTruth mt{};
        mt.sim_time_ns = t;
        mt.magnetic_field = {20000.0 + k, 0.0, 45000.0};
        sa::MagSample ms{};
        check(mag.step(mt, ms, error), "mag step");
        check(ms.updated == (k % 20 == 0), "mag measures every 20 ms");
        check(ms.value.x == 20000.0 + (k - k % 20), "mag value held between measurements");

        sa::BaroTruth bt{};
        bt.sim_time_ns = t;
        bt.altitude_m = 100.0 + k;
        sa::BaroSample bs{};
        check(baro.step(bt, bs, error), "baro step");
        check(bs.updated == (k % 10 == 0), "baro measures every 10 ms");
        check(bs.sample_time_ns == static_cast<std::uint64_t>(k - k % 10) * 1000000, "baro sample time");

        sa::GpsTruth gt{};
        gt.sim_time_ns = t;
        gt.position = {gps_setup.origin.latitude_deg, gps_setup.origin.longitude_deg, 121.321 + k};
        gt.velocity_nwu = {1.0, 0.0, 0.0};
        sa::GpsSample gs{};
        check(gps.step(gt, gs, error), "gps step");
        check(gs.updated == (k % 100 == 0), "gps measures every 100 ms");
        check(gs.position.altitude_m == 121.321 + (k - k % 100), "gps value held between measurements");
        check(gs.fix_type == 3 && gs.flags == 0, "gps fix and flags");
    }

    for (sa::ISensorAdapter* adapter : all) {
        std::vector<double> values{1.0};
        adapter->log_values(values);
        check(adapter->log_columns().empty() && values.empty(), "no log columns gives no values");
        check(adapter->reset(error), "reset through ISensorAdapter");
    }

    sa::BaroTruth late{};
    late.sim_time_ns = 7;
    sa::BaroSample bs{};
    check(!baro.step(late, bs, error) && !error.empty(), "a time jump is an error");

    if (failures == 0) {
        std::printf("mag_baro_gps_adapter_contract: OK\n");
    }
    return failures == 0 ? EXIT_SUCCESS : EXIT_FAILURE;
}
