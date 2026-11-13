#pragma once

// The reference implementation ("reference"): a LiDAR, a force-torque sensor and
// a joint sensor that return the true values, nothing else (no error of any
// kind). For hosts that have no other implementation, as the ideal baseline to
// compare implementations against, and as an example of an implementation. It depends on nothing
// but this library's interfaces and nlohmann/json (header-only); the LiDAR
// measures the world through the host's IRayCaster.
//
// A host links src/reference/reference_implementation.cpp (or the CMake target
// hakoniwa_sensor_adapter_reference) and registers it by name:
//
//   HAKO_SENSOR_ADAPTER_DECLARE_IMPLEMENTATION(reference);
//   const auto* reference = HAKO_SENSOR_ADAPTER_IMPLEMENTATION_ENTRY(reference)();

#include "hakoniwa/sensor_adapter/reference/reference_force_torque_adapter.hpp"
#include "hakoniwa/sensor_adapter/reference/reference_joint_adapter.hpp"
#include "hakoniwa/sensor_adapter/reference/reference_lidar_adapter.hpp"
#include "hakoniwa/sensor_adapter/sensor_adapter_implementation.hpp"

namespace hakoniwa::sensor_adapter::reference {

inline std::unique_ptr<ILidarSensorAdapter> create_lidar(const LidarAdapterSetup& setup, std::string& error)
{
    return ReferenceLidarAdapter::create(setup, error);
}

inline std::unique_ptr<IForceTorqueSensorAdapter> create_force_torque(const ForceTorqueAdapterSetup& setup,
                                                                      std::string& error)
{
    return ReferenceForceTorqueAdapter::create(setup, error);
}

inline std::unique_ptr<IJointSensorAdapter> create_joint(const JointSensorAdapterSetup& setup, std::string& error)
{
    return ReferenceJointAdapter::create(setup, error);
}

/// The descriptor of the reference implementation.
inline const SensorAdapterImplementation& implementation()
{
    static const SensorAdapterImplementation value = [] {
        SensorAdapterImplementation v;
        v.name = "reference";
        v.create_lidar = &create_lidar;
        v.create_force_torque = &create_force_torque;
        v.create_joint = &create_joint;
        return v;
    }();
    return value;
}

}  // namespace hakoniwa::sensor_adapter::reference

HAKO_SENSOR_ADAPTER_DECLARE_IMPLEMENTATION(reference);
