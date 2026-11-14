// Sensor logs for hosts: the configuration, the wrapped adapter, and the CSV and binary
// (HKBLOG version 1) files, against values computed here (docs/spec/sensor-log.md).
#include "hakoniwa/sensor_adapter/host/sensor_log.hpp"

#include <cstdio>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <string>
#include <vector>

namespace sa = hakoniwa::sensor_adapter;
namespace host = hakoniwa::sensor_adapter::host;
namespace fs = std::filesystem;

namespace {

int failures = 0;

void check(bool condition, const char* what)
{
    if (!condition) {
        std::fprintf(stderr, "FAIL: %s\n", what);
        ++failures;
    }
}

void check_eq(const std::string& actual, const std::string& expected, const char* what)
{
    if (actual != expected) {
        std::fprintf(stderr, "FAIL: %s:\n  got      %s\n  expected %s\n", what, actual.c_str(), expected.c_str());
        ++failures;
    }
}

// An adapter whose output is the truth plus 1, measured on every second step.
class FakeImu final : public sa::IImuSensorAdapter {
public:
    static std::string last_config_path;
    bool reset(std::string&) override { return true; }
    const std::vector<sa::LogColumn>& log_columns() const override { return columns_; }
    void log_values(std::vector<double>& values) const override { values = {static_cast<double>(steps_)}; }
    bool step(const sa::ImuTruth& truth, sa::ImuSample& out, std::string&) override
    {
        const bool measure = steps_ % 2 == 0;
        ++steps_;
        out.gyro.updated = measure;
        out.accel.updated = measure;
        if (measure) {
            out.gyro.value = {truth.angular_velocity.x + 1, truth.angular_velocity.y + 1, truth.angular_velocity.z + 1};
            out.accel.value = {truth.specific_force.x + 1, truth.specific_force.y + 1, truth.specific_force.z + 1};
        }
        return true;
    }
    sa::ImuAdapterCapabilities capabilities() const override { return {}; }
    bool calibration(sa::ImuCalibration&) const override { return false; }

private:
    std::vector<sa::LogColumn> columns_{{"model_steps", ""}};
    int steps_{0};
};
std::string FakeImu::last_config_path;

std::unique_ptr<sa::IImuSensorAdapter> create_fake_imu(const sa::ImuAdapterSetup& setup, std::string&)
{
    FakeImu::last_config_path = setup.config_path;
    return std::make_unique<FakeImu>();
}

class FakeBaro final : public sa::IBaroSensorAdapter {
public:
    bool reset(std::string&) override { return true; }
    const std::vector<sa::LogColumn>& log_columns() const override { return columns_; }
    void log_values(std::vector<double>& values) const override { values.clear(); }
    bool step(const sa::BaroTruth& truth, sa::BaroSample& out, std::string&) override
    {
        out.updated = true;
        out.pressure_pa = 100000.0 - truth.altitude_m;
        return true;
    }
    sa::BaroAdapterCapabilities capabilities() const override { return {}; }

private:
    std::vector<sa::LogColumn> columns_;
};

std::unique_ptr<sa::IBaroSensorAdapter> create_fake_baro(const sa::BaroAdapterSetup&, std::string&)
{
    return std::make_unique<FakeBaro>();
}

std::string write_file(const fs::path& path, const std::string& text)
{
    std::ofstream(path) << text;
    return path.string();
}

std::vector<std::string> lines(const fs::path& path)
{
    std::ifstream in(path);
    std::vector<std::string> out;
    for (std::string line; std::getline(in, line);) out.push_back(line);
    return out;
}

void step_imu(sa::IImuSensorAdapter& imu, int steps)
{
    std::string error;
    for (int i = 0; i < steps; ++i) {
        sa::ImuTruth truth;
        truth.sim_time_ns = static_cast<std::uint64_t>(i) * 500000U;  // 0.5 ms steps
        truth.angular_velocity = {0.1 * i, 0.2 * i, 0.3 * i};
        truth.specific_force = {i * 1.0, i * 2.0, -9.80665};
        sa::ImuSample out;
        check(imu.step(truth, out, error), "the wrapped step succeeds");
    }
}

// [LOG-CFG-010] a file without "model" is the implementation's own
void test_model_own_file(const fs::path& dir)
{
    sa::ImuAdapterSetup setup;
    setup.config_path = (dir / "model.json").string();
    std::string error;
    auto imu = host::create_logged_imu_sensor_adapter(setup, error, &create_fake_imu);
    check(imu != nullptr, "own file: created");
    check(dynamic_cast<FakeImu*>(imu.get()) != nullptr, "own file: not wrapped");
    check_eq(FakeImu::last_config_path, setup.config_path, "own file: passed as it is");
}

// [LOG-CFG-020] "model" without "log": the model's file, relative to the configuration
void test_model_without_log(const fs::path& dir)
{
    sa::ImuAdapterSetup setup;
    setup.config_path = write_file(dir / "sensor.json", R"({"model": "model.json"})");
    std::string error;
    auto imu = host::create_logged_imu_sensor_adapter(setup, error, &create_fake_imu);
    check(imu != nullptr, "no log: created");
    check(dynamic_cast<FakeImu*>(imu.get()) != nullptr, "no log: not wrapped");
    check_eq(FakeImu::last_config_path, (dir / "model.json").lexically_normal().string(), "no log: model resolved");
}

// [LOG-CFG-030 LOG-COL-010 LOG-COL-020 LOG-COL-030 LOG-FMT-010]
void test_csv(const fs::path& dir)
{
    sa::ImuAdapterSetup setup;
    setup.config_path = write_file(dir / "sensor.json", R"({"model": "model.json", "log": {"path": "logs/imu.csv"}})");
    std::string error;
    auto imu = host::create_logged_imu_sensor_adapter(setup, error, &create_fake_imu);
    check(imu != nullptr, "csv: created");
    if (!imu) return;
    check(imu->log_columns().size() == 1U, "csv: the adapter's own columns, unchanged");
    step_imu(*imu, 5);
    imu.reset();  // closes the file

    const auto rows = lines(dir / "logs" / "imu.csv");
    check(rows.size() == 6U, "csv: header + one row per step");
    if (rows.size() != 6U) return;
    check_eq(rows[0],
             "time_usec,truth_angular_velocity_x,truth_angular_velocity_y,truth_angular_velocity_z,"
             "truth_specific_force_x,truth_specific_force_y,truth_specific_force_z,gyro_x,gyro_y,gyro_z,"
             "accel_x,accel_y,accel_z,gyro_updated,accel_updated,gyro_flags,accel_flags,model_steps",
             "csv: header");
    // step 2 at 1000 us: truth gyro (0.2, 0.4, 0.6), force (2, 4, -9.80665); measured: +1
    check_eq(rows[3], "1000,0.2,0.4,0.6,2,4,-9.80665,1.2,1.4,1.6,3,5,-8.80665,1,1,0,0,3", "csv: a measured row");
    // step 3 at 1500 us: not measured
    check_eq(rows[4].substr(0, 5), "1500,", "csv: time of an unmeasured row");
    check(rows[4].size() >= 10U && rows[4].substr(rows[4].size() - 10) == ",0,0,0,0,4", "csv: an unmeasured row");
}

// [LOG-FMT-020]
void test_binary(const fs::path& dir)
{
    sa::ImuAdapterSetup setup;
    setup.config_path =
        write_file(dir / "sensor.json", R"({"model": "model.json", "log": {"path": "imu.bin", "format": "binary"}})");
    std::string error;
    auto imu = host::create_logged_imu_sensor_adapter(setup, error, &create_fake_imu);
    check(imu != nullptr, "binary: created");
    if (!imu) return;
    step_imu(*imu, 3);
    imu.reset();

    std::ifstream in(dir / "imu.bin", std::ios::binary);
    const std::vector<char> bytes((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
    std::size_t at = 8;
    bool ok = bytes.size() >= 8 && std::memcmp(bytes.data(), "HKBLOG\0\0", 8) == 0;
    check(ok, "binary: magic");
    const auto take = [&](auto& value) {
        if (at + sizeof(value) > bytes.size()) {
            ok = false;
            return;
        }
        std::memcpy(&value, bytes.data() + at, sizeof(value));
        at += sizeof(value);
    };
    std::uint16_t version = 0, flags = 0;
    std::uint32_t count = 0;
    take(version);
    take(flags);
    take(count);
    check(ok && version == 1 && flags == 0, "binary: version 1, flags 0");
    check(count == 18U, "binary: time + 16 wrapper columns + 1 adapter column");
    for (std::uint32_t i = 0; ok && i < count; ++i) {
        std::uint8_t type = 0;
        std::uint16_t length = 0;
        take(type);
        take(length);
        check(type == (i == 0 ? 8 : 10), "binary: uint64 time, double values");
        at += length;
    }
    for (int row = 0; ok && row < 3; ++row) {
        std::uint32_t size = 0;
        std::uint64_t time = 0;
        double truth_gx = 0;
        take(size);
        take(time);
        take(truth_gx);
        check(size == 8U + 17U * 8U, "binary: row size");
        check(time == static_cast<std::uint64_t>(row) * 500U, "binary: row time");
        check(truth_gx == 0.1 * row, "binary: first value");
        at += 16U * 8U;
    }
    check(ok && at == bytes.size(), "binary: three rows and nothing else");
}

// [LOG-COL-020]
void test_baro(const fs::path& dir)
{
    sa::BaroAdapterSetup setup;
    setup.config_path =
        write_file(dir / "sensor.json", R"({"model": "model.json", "log": {"path": "baro.csv", "format": "csv"}})");
    std::string error;
    auto baro = host::create_logged_baro_sensor_adapter(setup, error, &create_fake_baro);
    check(baro != nullptr, "baro: created");
    if (!baro) return;
    sa::BaroTruth truth;
    truth.altitude_m = 120.5;
    sa::BaroSample out;
    check(baro->step(truth, out, error), "baro: step");
    baro.reset();
    const auto rows = lines(dir / "baro.csv");
    check(rows.size() == 2U, "baro: header + one row");
    if (rows.size() == 2U) check_eq(rows[1], "0,120.5,101325,15,99879.5,0,0,1,0", "baro: truth beside the output");
}

// [LOG-CFG-040]
void test_invalid(const fs::path& dir)
{
    const auto refused = [&](const std::string& text, const char* expect, const char* what) {
        sa::ImuAdapterSetup setup;
        setup.config_path = write_file(dir / "sensor.json", text);
        std::string error;
        const bool none = host::create_logged_imu_sensor_adapter(setup, error, &create_fake_imu) == nullptr;
        check(none && error.find(expect) != std::string::npos, what);
    };
    refused(R"({"model": "model.json", "log": {"path": "x.csv", "format": "xml"}})", "csv or binary", "bad format");
    refused(R"({"model": "model.json", "logs": {}})", "unknown key logs", "unknown key");
    refused(R"({"model": "model.json", "log": {"format": "csv"}})", "path", "log without a path");
    sa::ImuAdapterSetup setup;
    setup.config_path = (dir / "missing.json").string();
    std::string error;
    check(host::create_logged_imu_sensor_adapter(setup, error, &create_fake_imu) == nullptr &&
              error.find("cannot open") != std::string::npos,
          "missing file");
}

}  // namespace

int main()
{
    const fs::path dir = fs::temp_directory_path() / "hakoniwa_sensor_log_test";
    using Test = void (*)(const fs::path&);
    for (const Test test : {test_model_own_file, test_model_without_log, test_csv, test_binary, test_baro, test_invalid}) {
        fs::remove_all(dir);
        fs::create_directories(dir);
        write_file(dir / "model.json", "{}");
        test(dir);
    }
    fs::remove_all(dir);
    if (failures != 0) {
        std::fprintf(stderr, "%d failure(s)\n", failures);
        return 1;
    }
    std::printf("sensor_log: all checks passed\n");
    return 0;
}
