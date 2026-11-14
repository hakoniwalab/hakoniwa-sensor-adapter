#pragma once
// Sensor logs for hosts: what each IMU, magnetometer, barometer and GPS adapter received
// and gave on every physics step, whatever its implementation (docs/spec/sensor-log.md).
//
// A sensor whose configuration file is a sensor log configuration
//
//   {"model": "<the model's configuration>", "log": {"path": "<file>", "format": "csv" | "binary"}}
//
// is created from the model's configuration "model" and the host writes one row per physics step to "log.path"
// (relative to that file): the time, the truth it received, its output, then the
// model's own log columns. Without "log" it writes nothing. A configuration file
// without "model" is the model's own and is used as it is.
#include <cstdint>
#include <fstream>
#include <memory>
#include <string>
#include <vector>

#include "hakoniwa/sensor_adapter/sensor_adapter_implementation.hpp"

namespace hakoniwa::sensor_adapter::host {

/// The file formats: CSV, or Hakoniwa Drone's binary log (HKBLOG version 1).
enum class SensorLogFormat { Csv, Binary };

/// One log file: a uint64 time column followed by double columns.
class SensorLogFile {
public:
    /// Opens (truncates) the file and writes its header. Returns nullptr and sets error on failure.
    static std::unique_ptr<SensorLogFile> create(const std::string& path, SensorLogFormat format,
                                                 const std::string& time_column,
                                                 const std::vector<std::string>& columns, std::string& error);
    ~SensorLogFile();
    /// One row: the time and one value per column.
    void write(std::uint64_t time, const std::vector<double>& values);
    void flush();

private:
    SensorLogFile() = default;
    std::ofstream out_;
    SensorLogFormat format_{SensorLogFormat::Csv};
    std::size_t columns_{0};
    std::vector<std::uint8_t> buffer_;
    std::string line_;
    std::uint32_t unflushed_rows_{0};
};

/// A sensor log configuration read from a sensor's configuration file.
struct SensorLogConfig {
    /// The model's configuration file (absolute or relative to the working directory).
    std::string model_config_path;
    /// Whether to log, where and how.
    bool log{false};
    std::string log_path;
    SensorLogFormat format{SensorLogFormat::Csv};
};

/// Reads config_path. Without "model" the file is the model's own: model_config_path is
/// config_path and log is false. Returns false and sets error when the file is not valid.
bool read_sensor_log_config(const std::string& config_path, SensorLogConfig& config, std::string& error);

/// An adapter from model (an implementation's factory) wrapped with its sensor log: a host
/// calls these in place of the implementation's factory. With a file that has no "model",
/// the implementation's adapter itself. nullptr and error on failure.
std::unique_ptr<sensor_adapter::IImuSensorAdapter> create_logged_imu_sensor_adapter(
    const sensor_adapter::ImuAdapterSetup& setup, std::string& error,
    sensor_adapter::SensorAdapterFactory<sensor_adapter::IImuSensorAdapter, sensor_adapter::ImuAdapterSetup> model);
std::unique_ptr<sensor_adapter::IMagSensorAdapter> create_logged_mag_sensor_adapter(
    const sensor_adapter::MagAdapterSetup& setup, std::string& error,
    sensor_adapter::SensorAdapterFactory<sensor_adapter::IMagSensorAdapter, sensor_adapter::MagAdapterSetup> model);
std::unique_ptr<sensor_adapter::IBaroSensorAdapter> create_logged_baro_sensor_adapter(
    const sensor_adapter::BaroAdapterSetup& setup, std::string& error,
    sensor_adapter::SensorAdapterFactory<sensor_adapter::IBaroSensorAdapter, sensor_adapter::BaroAdapterSetup> model);
std::unique_ptr<sensor_adapter::IGpsSensorAdapter> create_logged_gps_sensor_adapter(
    const sensor_adapter::GpsAdapterSetup& setup, std::string& error,
    sensor_adapter::SensorAdapterFactory<sensor_adapter::IGpsSensorAdapter, sensor_adapter::GpsAdapterSetup> model);

}  // namespace hakoniwa::sensor_adapter::host
