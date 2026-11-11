#pragma once

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "hakoniwa/sensor_adapter/baro_sensor_adapter.hpp"
#include "hakoniwa/sensor_adapter/camera_sensor_adapter.hpp"
#include "hakoniwa/sensor_adapter/force_torque_sensor_adapter.hpp"
#include "hakoniwa/sensor_adapter/gps_sensor_adapter.hpp"
#include "hakoniwa/sensor_adapter/imu_sensor_adapter.hpp"
#include "hakoniwa/sensor_adapter/joint_sensor_adapter.hpp"
#include "hakoniwa/sensor_adapter/lidar_sensor_adapter.hpp"
#include "hakoniwa/sensor_adapter/mag_sensor_adapter.hpp"
#include "hakoniwa/sensor_adapter/rangefinder_sensor_adapter.hpp"

namespace hakoniwa::sensor_adapter {

/// Version of SensorAdapterImplementation. A host refuses a descriptor with another version.
constexpr std::uint32_t kSensorAdapterImplementationVersion = 1;

/// Creates one adapter. Returns nullptr and sets error on failure.
template <class Adapter, class Setup>
using SensorAdapterFactory = std::unique_ptr<Adapter> (*)(const Setup& setup, std::string& error);

/**
 * @brief How a host obtains adapters from one implementation.
 *
 * One descriptor per implementation (for example "hakoniwa", "gazebo", "airsim").
 * A sensor type the implementation does not provide is nullptr, never a stub that
 * always fails, so a host can tell "not provided" from "failed to create".
 *
 * The host and the implementation are built with the same compiler and C++ standard
 * library, as for every C++ interface here.
 */
struct SensorAdapterImplementation {
    std::uint32_t version{kSensorAdapterImplementationVersion};
    /// Implementation name: a C identifier, the same as in the entry point name.
    const char* name{nullptr};

    SensorAdapterFactory<IImuSensorAdapter, ImuAdapterSetup> create_imu{nullptr};
    SensorAdapterFactory<IMagSensorAdapter, MagAdapterSetup> create_mag{nullptr};
    SensorAdapterFactory<IBaroSensorAdapter, BaroAdapterSetup> create_baro{nullptr};
    SensorAdapterFactory<IGpsSensorAdapter, GpsAdapterSetup> create_gps{nullptr};
    SensorAdapterFactory<IRangefinderSensorAdapter, RangefinderAdapterSetup> create_rangefinder{nullptr};
    SensorAdapterFactory<ILidarSensorAdapter, LidarAdapterSetup> create_lidar{nullptr};
    SensorAdapterFactory<IDepthCameraSensorAdapter, DepthCameraAdapterSetup> create_depth_camera{nullptr};
    SensorAdapterFactory<IRgbCameraSensorAdapter, RgbCameraAdapterSetup> create_rgb_camera{nullptr};
    SensorAdapterFactory<IForceTorqueSensorAdapter, ForceTorqueAdapterSetup> create_force_torque{nullptr};
    SensorAdapterFactory<IJointSensorAdapter, JointSensorAdapterSetup> create_joint{nullptr};
};

/**
 * @brief The sensor types a descriptor provides, in a fixed order.
 *
 * Type names: imu, mag, baro, gps, rangefinder, lidar, depth_camera, rgb_camera,
 * force_torque, joint.
 */
inline std::vector<std::string> provided_sensor_types(const SensorAdapterImplementation& implementation)
{
    std::vector<std::string> types;
    if (implementation.create_imu) types.emplace_back("imu");
    if (implementation.create_mag) types.emplace_back("mag");
    if (implementation.create_baro) types.emplace_back("baro");
    if (implementation.create_gps) types.emplace_back("gps");
    if (implementation.create_rangefinder) types.emplace_back("rangefinder");
    if (implementation.create_lidar) types.emplace_back("lidar");
    if (implementation.create_depth_camera) types.emplace_back("depth_camera");
    if (implementation.create_rgb_camera) types.emplace_back("rgb_camera");
    if (implementation.create_force_torque) types.emplace_back("force_torque");
    if (implementation.create_joint) types.emplace_back("joint");
    return types;
}

}  // namespace hakoniwa::sensor_adapter

/**
 * @brief Entry point of an implementation: hako_sensor_adapter_implementation_<name>().
 *
 * Each implementation library defines exactly one entry point, whose name is fixed by
 * its implementation name. A host can therefore register an implementation from its
 * name alone, for example from a build option that lists the linked implementations:
 *
 * @code
 * // in the implementation library
 * HAKO_SENSOR_ADAPTER_DEFINE_IMPLEMENTATION(gazebo)
 * {
 *     static const hakoniwa::sensor_adapter::SensorAdapterImplementation implementation = [] {
 *         hakoniwa::sensor_adapter::SensorAdapterImplementation value;
 *         value.name = "gazebo";
 *         value.create_force_torque = &create_gz_force_torque_sensor_adapter;
 *         return value;
 *     }();
 *     return &implementation;
 * }
 *
 * // in the host
 * HAKO_SENSOR_ADAPTER_DECLARE_IMPLEMENTATION(gazebo);
 * const auto* gazebo = HAKO_SENSOR_ADAPTER_IMPLEMENTATION_ENTRY(gazebo)();
 * @endcode
 *
 * The entry point has C linkage only to fix its symbol name; it returns a C++ struct.
 */
#define HAKO_SENSOR_ADAPTER_IMPLEMENTATION_ENTRY(name) hako_sensor_adapter_implementation_##name

#define HAKO_SENSOR_ADAPTER_DECLARE_IMPLEMENTATION(name) \
    extern "C" const ::hakoniwa::sensor_adapter::SensorAdapterImplementation* \
    HAKO_SENSOR_ADAPTER_IMPLEMENTATION_ENTRY(name)()

#define HAKO_SENSOR_ADAPTER_DEFINE_IMPLEMENTATION(name) \
    HAKO_SENSOR_ADAPTER_DECLARE_IMPLEMENTATION(name); \
    extern "C" const ::hakoniwa::sensor_adapter::SensorAdapterImplementation* \
    HAKO_SENSOR_ADAPTER_IMPLEMENTATION_ENTRY(name)()
