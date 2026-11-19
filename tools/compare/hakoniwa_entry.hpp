// The Hakoniwa implementation for the comparisons, reached through the common interface by its name.
//
// The comparisons do not see the Hakoniwa sources: they declare the "hakoniwa" entry point
// (sensor_adapter_implementation.hpp) and link the libraries of a hakoniwa-sensor-pro release archive, which
// defines it. The functions below keep the names the comparisons call.
#pragma once

#include <memory>
#include <string>

#include "hakoniwa/sensor_adapter/sensor_adapter_implementation.hpp"

HAKO_SENSOR_ADAPTER_DECLARE_IMPLEMENTATION(hakoniwa);

namespace hs {

inline const hakoniwa::sensor_adapter::SensorAdapterImplementation& hakoniwa_implementation()
{
    return *HAKO_SENSOR_ADAPTER_IMPLEMENTATION_ENTRY(hakoniwa)();
}

inline std::unique_ptr<hakoniwa::sensor_adapter::IImuSensorAdapter> create_hakoniwa_imu_sensor_adapter(
    const hakoniwa::sensor_adapter::ImuAdapterSetup& setup, std::string& error)
{
    return hakoniwa_implementation().create_imu(setup, error);
}

inline std::unique_ptr<hakoniwa::sensor_adapter::IMagSensorAdapter> create_hakoniwa_mag_sensor_adapter(
    const hakoniwa::sensor_adapter::MagAdapterSetup& setup, std::string& error)
{
    return hakoniwa_implementation().create_mag(setup, error);
}

inline std::unique_ptr<hakoniwa::sensor_adapter::IBaroSensorAdapter> create_hakoniwa_baro_sensor_adapter(
    const hakoniwa::sensor_adapter::BaroAdapterSetup& setup, std::string& error)
{
    return hakoniwa_implementation().create_baro(setup, error);
}

inline std::unique_ptr<hakoniwa::sensor_adapter::IGpsSensorAdapter> create_hakoniwa_gps_sensor_adapter(
    const hakoniwa::sensor_adapter::GpsAdapterSetup& setup, std::string& error)
{
    return hakoniwa_implementation().create_gps(setup, error);
}

}  // namespace hs
