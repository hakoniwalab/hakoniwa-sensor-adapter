// Sensor logs for hosts (include/hakoniwa/sensor_adapter/host/sensor_log.hpp).
#include "hakoniwa/sensor_adapter/host/sensor_log.hpp"

#include <algorithm>
#include <array>
#include <cstdio>
#include <cstring>
#include <filesystem>
#include <limits>
#include <stdexcept>
#include <type_traits>

#include <nlohmann/json.hpp>

namespace hakoniwa::sensor_adapter::host {

namespace sa = hakoniwa::sensor_adapter;
namespace fs = std::filesystem;

namespace {

// HKBLOG version 1, as Hakoniwa Drone writes it: column types are its LogType values.
constexpr std::uint8_t kHkblogUint64 = 8;
constexpr std::uint8_t kHkblogDouble = 10;
// A run may be stopped without its cleanup: rows reach the file at least this often.
constexpr std::uint32_t kFlushRows = 256;

template <typename T>
void append_le(std::vector<std::uint8_t>& out, T value)
{
    static_assert(std::is_integral_v<T> || std::is_floating_point_v<T>);
    std::array<std::uint8_t, sizeof(T)> bytes{};
    std::memcpy(bytes.data(), &value, sizeof(T));
#if defined(__BYTE_ORDER__) && __BYTE_ORDER__ == __ORDER_BIG_ENDIAN__
    std::reverse(bytes.begin(), bytes.end());
#endif
    out.insert(out.end(), bytes.begin(), bytes.end());
}

void append_column(std::vector<std::uint8_t>& out, std::uint8_t type, const std::string& name)
{
    if (name.size() > std::numeric_limits<std::uint16_t>::max()) {
        throw std::runtime_error("log column name is too long: " + name);
    }
    out.push_back(type);
    append_le<std::uint16_t>(out, static_cast<std::uint16_t>(name.size()));
    out.insert(out.end(), name.begin(), name.end());
}

}  // namespace

std::unique_ptr<SensorLogFile> SensorLogFile::create(const std::string& path, SensorLogFormat format,
                                                     const std::string& time_column,
                                                     const std::vector<std::string>& columns, std::string& error)
{
    std::unique_ptr<SensorLogFile> file(new SensorLogFile());
    try {
        const fs::path parent = fs::path(path).parent_path();
        if (!parent.empty()) fs::create_directories(parent);
        file->out_.open(path, std::ios::binary | std::ios::trunc);
        if (!file->out_) {
            error = "cannot write " + path;
            return nullptr;
        }
        file->format_ = format;
        file->columns_ = columns.size();
        if (format == SensorLogFormat::Csv) {
            std::string header = time_column;
            for (const auto& column : columns) header += "," + column;
            header += "\n";
            file->out_ << header;
        } else {
            std::vector<std::uint8_t> bytes{'H', 'K', 'B', 'L', 'O', 'G', 0, 0};
            append_le<std::uint16_t>(bytes, 1);  // version
            append_le<std::uint16_t>(bytes, 0);  // flags
            append_le<std::uint32_t>(bytes, static_cast<std::uint32_t>(columns.size() + 1));
            append_column(bytes, kHkblogUint64, time_column);
            for (const auto& column : columns) append_column(bytes, kHkblogDouble, column);
            file->out_.write(reinterpret_cast<const char*>(bytes.data()), static_cast<std::streamsize>(bytes.size()));
        }
        if (!file->out_) {
            error = "cannot write " + path;
            return nullptr;
        }
    } catch (const std::exception& e) {
        error = e.what();
        return nullptr;
    }
    return file;
}

SensorLogFile::~SensorLogFile()
{
    flush();
}

void SensorLogFile::write(std::uint64_t time, const std::vector<double>& values)
{
    if (values.size() != columns_) {
        throw std::runtime_error("log row has " + std::to_string(values.size()) + " values for " +
                                 std::to_string(columns_) + " columns");
    }
    if (format_ == SensorLogFormat::Csv) {
        char number[40];
        line_.clear();
        std::snprintf(number, sizeof(number), "%llu", static_cast<unsigned long long>(time));
        line_ += number;
        for (const double value : values) {
            std::snprintf(number, sizeof(number), ",%.9g", value);
            line_ += number;
        }
        line_ += "\n";
        out_ << line_;
    } else {
        append_le<std::uint32_t>(buffer_, static_cast<std::uint32_t>(sizeof(std::uint64_t) + values.size() * sizeof(double)));
        append_le<std::uint64_t>(buffer_, time);
        for (const double value : values) append_le<double>(buffer_, value);
    }
    if (++unflushed_rows_ >= kFlushRows) flush();
}

void SensorLogFile::flush()
{
    if (!out_.is_open()) return;
    if (!buffer_.empty()) {
        out_.write(reinterpret_cast<const char*>(buffer_.data()), static_cast<std::streamsize>(buffer_.size()));
        buffer_.clear();
    }
    out_.flush();
    unflushed_rows_ = 0;
}

bool read_sensor_log_config(const std::string& config_path, SensorLogConfig& config, std::string& error)
{
    config = SensorLogConfig{};
    try {
        std::ifstream in(config_path);
        if (!in) {
            error = "cannot open " + config_path;
            return false;
        }
        const nlohmann::json root = nlohmann::json::parse(in);
        if (!root.is_object() || !root.contains("model")) {
            config.model_config_path = config_path;  // the model's own file
            return true;
        }
        const fs::path base = fs::path(config_path).parent_path();
        const auto resolve = [&base](const std::string& path) {
            const fs::path p(path);
            return (p.is_absolute() || base.empty() ? p : base / p).lexically_normal().string();
        };
        for (const auto& item : root.items()) {
            if (item.key() != "model" && item.key() != "log") {
                error = config_path + ": unknown key " + item.key();
                return false;
            }
        }
        config.model_config_path = resolve(root.at("model").get<std::string>());
        if (root.contains("log")) {
            const auto& log = root["log"];
            for (const auto& item : log.items()) {
                if (item.key() != "path" && item.key() != "format") {
                    error = config_path + ": unknown key log." + item.key();
                    return false;
                }
            }
            config.log = true;
            config.log_path = resolve(log.at("path").get<std::string>());
            const std::string format = log.value("format", "csv");
            if (format == "csv") {
                config.format = SensorLogFormat::Csv;
            } else if (format == "binary") {
                config.format = SensorLogFormat::Binary;
            } else {
                error = config_path + ": log.format must be csv or binary";
                return false;
            }
        }
    } catch (const std::exception& e) {
        error = config_path + ": " + e.what();
        return false;
    }
    return true;
}

namespace {

void push(std::vector<double>& row, const sa::Vector3& v)
{
    row.push_back(v.x);
    row.push_back(v.y);
    row.push_back(v.z);
}

std::vector<std::string> xyz(const std::string& name)
{
    return {name + "_x", name + "_y", name + "_z"};
}

void append(std::vector<std::string>& to, const std::vector<std::string>& more)
{
    to.insert(to.end(), more.begin(), more.end());
}

// What every wrapper shares: the model, its log, and the row being built.
template <class Interface>
class LoggedAdapter : public Interface {
public:
    bool reset(std::string& error) override { return model_->reset(error); }
    const std::vector<sa::LogColumn>& log_columns() const override { return model_->log_columns(); }
    void log_values(std::vector<double>& values) const override { model_->log_values(values); }

    bool open(std::unique_ptr<Interface> model, const SensorLogConfig& config,
              std::vector<std::string> columns, std::string& error)
    {
        model_ = std::move(model);
        for (const auto& column : model_->log_columns()) columns.push_back(column.name);
        log_ = SensorLogFile::create(config.log_path, config.format, "time_usec", columns, error);
        return log_ != nullptr;
    }

protected:
    // The row after one step: the wrapper's values, then the model's own columns.
    void write_row(std::uint64_t sim_time_ns)
    {
        model_->log_values(model_values_);
        row_.insert(row_.end(), model_values_.begin(), model_values_.end());
        log_->write(sim_time_ns / 1000U, row_);
        row_.clear();
    }

    std::unique_ptr<Interface> model_;
    std::unique_ptr<SensorLogFile> log_;
    std::vector<double> row_;
    std::vector<double> model_values_;
};

class LoggedImuAdapter final : public LoggedAdapter<sa::IImuSensorAdapter> {
public:
    static std::vector<std::string> columns()
    {
        std::vector<std::string> c;
        append(c, xyz("truth_angular_velocity"));
        append(c, xyz("truth_specific_force"));
        append(c, xyz("gyro"));
        append(c, xyz("accel"));
        append(c, {"gyro_updated", "accel_updated", "gyro_flags", "accel_flags"});
        return c;
    }
    bool step(const sa::ImuTruth& truth, sa::ImuSample& out, std::string& error) override
    {
        if (!model_->step(truth, out, error)) return false;
        push(row_, truth.angular_velocity);
        push(row_, truth.specific_force);
        push(row_, out.gyro.value);
        push(row_, out.accel.value);
        row_.push_back(out.gyro.updated ? 1.0 : 0.0);
        row_.push_back(out.accel.updated ? 1.0 : 0.0);
        row_.push_back(static_cast<double>(out.gyro.flags));
        row_.push_back(static_cast<double>(out.accel.flags));
        write_row(truth.sim_time_ns);
        return true;
    }
    sa::ImuAdapterCapabilities capabilities() const override { return model_->capabilities(); }
    bool calibration(sa::ImuCalibration& out) const override { return model_->calibration(out); }
};

class LoggedMagAdapter final : public LoggedAdapter<sa::IMagSensorAdapter> {
public:
    static std::vector<std::string> columns()
    {
        std::vector<std::string> c;
        append(c, xyz("truth_magnetic_field"));
        append(c, xyz("magnetic_field"));
        append(c, {"updated", "flags"});
        return c;
    }
    bool step(const sa::MagTruth& truth, sa::MagSample& out, std::string& error) override
    {
        if (!model_->step(truth, out, error)) return false;
        push(row_, truth.magnetic_field);
        push(row_, out.value);
        row_.push_back(out.updated ? 1.0 : 0.0);
        row_.push_back(static_cast<double>(out.flags));
        write_row(truth.sim_time_ns);
        return true;
    }
    sa::MagAdapterCapabilities capabilities() const override { return model_->capabilities(); }
};

class LoggedBaroAdapter final : public LoggedAdapter<sa::IBaroSensorAdapter> {
public:
    static std::vector<std::string> columns()
    {
        return {"truth_altitude_m", "truth_sea_level_pressure_pa", "truth_sea_level_temperature_c",
                "pressure_pa", "temperature_c", "pressure_altitude_m", "updated", "flags"};
    }
    bool step(const sa::BaroTruth& truth, sa::BaroSample& out, std::string& error) override
    {
        if (!model_->step(truth, out, error)) return false;
        row_.push_back(truth.altitude_m);
        row_.push_back(truth.sea_level_pressure_pa);
        row_.push_back(truth.sea_level_temperature_c);
        row_.push_back(out.pressure_pa);
        row_.push_back(out.temperature_c);
        row_.push_back(out.pressure_altitude_m);
        row_.push_back(out.updated ? 1.0 : 0.0);
        row_.push_back(static_cast<double>(out.flags));
        write_row(truth.sim_time_ns);
        return true;
    }
    sa::BaroAdapterCapabilities capabilities() const override { return model_->capabilities(); }
};

class LoggedGpsAdapter final : public LoggedAdapter<sa::IGpsSensorAdapter> {
public:
    static std::vector<std::string> columns()
    {
        std::vector<std::string> c{"truth_latitude_deg", "truth_longitude_deg", "truth_altitude_m"};
        append(c, xyz("truth_position_nwu"));
        append(c, xyz("truth_velocity_nwu"));
        append(c, {"latitude_deg", "longitude_deg", "altitude_m"});
        append(c, xyz("velocity_nwu"));
        append(c, {"ground_speed_mps", "course_deg", "course_valid", "horizontal_accuracy_m",
                   "vertical_accuracy_m", "speed_accuracy_mps", "satellites_visible", "fix_type", "updated",
                   "flags"});
        return c;
    }
    bool step(const sa::GpsTruth& truth, sa::GpsSample& out, std::string& error) override
    {
        if (!model_->step(truth, out, error)) return false;
        row_.push_back(truth.position.latitude_deg);
        row_.push_back(truth.position.longitude_deg);
        row_.push_back(truth.position.altitude_m);
        push(row_, truth.position_nwu);
        push(row_, truth.velocity_nwu);
        row_.push_back(out.position.latitude_deg);
        row_.push_back(out.position.longitude_deg);
        row_.push_back(out.position.altitude_m);
        push(row_, out.velocity_nwu);
        row_.push_back(out.ground_speed_mps);
        row_.push_back(out.course_deg);
        row_.push_back(out.course_valid ? 1.0 : 0.0);
        row_.push_back(out.horizontal_accuracy_m);
        row_.push_back(out.vertical_accuracy_m);
        row_.push_back(out.speed_accuracy_mps);
        row_.push_back(static_cast<double>(out.satellites_visible));
        row_.push_back(static_cast<double>(out.fix_type));
        row_.push_back(out.updated ? 1.0 : 0.0);
        row_.push_back(static_cast<double>(out.flags));
        write_row(truth.sim_time_ns);
        return true;
    }
    sa::GpsAdapterCapabilities capabilities() const override { return model_->capabilities(); }
};

// The model's adapter from the model's configuration, wrapped when the file asks for a log.
template <class Logged, class Interface, class Setup>
std::unique_ptr<Interface> create_logged(const Setup& setup, std::string& error,
                                         sa::SensorAdapterFactory<Interface, Setup> model)
{
    if (model == nullptr) {
        error = "no model adapter";
        return nullptr;
    }
    SensorLogConfig config;
    if (!read_sensor_log_config(setup.config_path, config, error)) return nullptr;
    Setup model_setup = setup;
    model_setup.config_path = config.model_config_path;
    auto adapter = model(model_setup, error);
    if (!adapter || !config.log) return adapter;
    auto logged = std::make_unique<Logged>();
    if (!logged->open(std::move(adapter), config, Logged::columns(), error)) return nullptr;
    return logged;
}

}  // namespace

std::unique_ptr<sa::IImuSensorAdapter> create_logged_imu_sensor_adapter(
    const sa::ImuAdapterSetup& setup, std::string& error,
    sa::SensorAdapterFactory<sa::IImuSensorAdapter, sa::ImuAdapterSetup> model)
{
    return create_logged<LoggedImuAdapter>(setup, error, model);
}

std::unique_ptr<sa::IMagSensorAdapter> create_logged_mag_sensor_adapter(
    const sa::MagAdapterSetup& setup, std::string& error,
    sa::SensorAdapterFactory<sa::IMagSensorAdapter, sa::MagAdapterSetup> model)
{
    return create_logged<LoggedMagAdapter>(setup, error, model);
}

std::unique_ptr<sa::IBaroSensorAdapter> create_logged_baro_sensor_adapter(
    const sa::BaroAdapterSetup& setup, std::string& error,
    sa::SensorAdapterFactory<sa::IBaroSensorAdapter, sa::BaroAdapterSetup> model)
{
    return create_logged<LoggedBaroAdapter>(setup, error, model);
}

std::unique_ptr<sa::IGpsSensorAdapter> create_logged_gps_sensor_adapter(
    const sa::GpsAdapterSetup& setup, std::string& error,
    sa::SensorAdapterFactory<sa::IGpsSensorAdapter, sa::GpsAdapterSetup> model)
{
    return create_logged<LoggedGpsAdapter>(setup, error, model);
}

}  // namespace hakoniwa::sensor_adapter::host
