#pragma once

#include <cstdint>

#include "hakoniwa/sensor_adapter/baro_sensor_adapter.hpp"
#include "hakoniwa/sensor_adapter/gps_sensor_adapter.hpp"
#include "hakoniwa/sensor_adapter/imu_sensor_adapter.hpp"
#include "hakoniwa/sensor_adapter/mag_sensor_adapter.hpp"

namespace hakoniwa::sensor_adapter::host {

/**
 * @brief Motion of a rigid body's origin.
 *
 * The world frame is NWU and the body frame is FLU. The quaternion rotates
 * vector coordinates from world axes to body axes. Angular quantities are
 * expressed in body axes. linear_acceleration_nwu is kinematic acceleration;
 * gravity is not included.
 */
struct RigidBodyMotion {
    Vector3 position_nwu{};                 ///< [m]
    Vector3 velocity_nwu{};                 ///< [m/s]
    Quaternion orientation_world_to_body{};
    Vector3 angular_velocity_body{};        ///< [rad/s]
    Vector3 angular_acceleration_body{};    ///< [rad/s^2]
    Vector3 linear_acceleration_nwu{};      ///< [m/s^2]
};

/**
 * @brief Fixed pose of a sensor relative to the rigid-body origin.
 *
 * position_body_m is expressed in body FLU axes. The quaternion rotates
 * vector coordinates from body axes to sensor axes.
 */
struct MountPose {
    Vector3 position_body_m{};
    Quaternion orientation_body_to_sensor{};
};

/**
 * @brief Rotor-vibration inputs at the IMU point, expressed in body FLU.
 *
 * These observation disturbances default to zero. They do not change the
 * rigid-body motion and are not used in the lever-arm kinematics.
 */
struct ImuVibration {
    Vector3 acceleration_body{};          ///< [m/s^2]
    Vector3 angular_velocity_body{};      ///< [rad/s]
};

/**
 * @brief Make a mount pose from roll, pitch and yaw in degrees.
 *
 * This uses the robot-runtime convention R_body_from_sensor =
 * Rz(yaw) * Ry(pitch) * Rx(roll): the angles describe the sensor axes in the
 * body frame. The returned quaternion is its inverse because MountPose stores
 * the coordinate rotation from body axes to sensor axes.
 */
MountPose mount_from_rpy_deg(
    const Vector3& position_body_m, double roll_deg, double pitch_deg, double yaw_deg);

/**
 * @brief Convert a Drone PRO body-FRD mount to the equivalent FLU MountPose.
 *
 * Drone PRO defines R_body_from_sensor = Rz(yaw) * Ry(pitch) * Rx(roll) in
 * FRD. Conjugating that rotation by the FRD/FLU axis flip gives the same roll
 * with negated pitch and yaw in FLU; position y and z are negated as well.
 */
MountPose mount_from_frd_rpy_deg(
    const Vector3& position_frd_m, double roll_deg, double pitch_deg, double yaw_deg);

/// Geomagnetic field definition used by robot-runtime.
struct GeomagneticField {
    double intensity_nT{0.0};
    /// Declination is positive east of north [deg].
    double declination_deg{0.0};
    /// Inclination is positive down from horizontal [deg].
    double inclination_deg{0.0};
};

/// World values shared by the host truth builders.
struct World {
    Vector3 gravity_nwu{0.0, 0.0, -9.80665};
    GeomagneticField geomagnetic_field{};
    /// Geodetic position of the local NWU origin. altitude_m is WGS-84
    /// ellipsoid height h [m], as used by GPS.
    GeodeticPosition origin{};
    /// Geoid undulation N = ellipsoid height h - mean-sea-level height H [m].
    /// Thus the origin's mean-sea-level height is origin.altitude_m - N.
    double geoid_height_m{0.0};
    double sea_level_pressure_pa{101325.0};
    double sea_level_temperature_c{15.0};
};

/**
 * @brief Convert a point in the origin's local tangent NWU frame to WGS-84.
 *
 * The local displacement is rotated through ENU into ECEF and added to the
 * origin's ECEF coordinate. Altitude is ellipsoid height; no geoid model is
 * applied.
 */
GeodeticPosition nwu_to_geodetic(
    const Vector3& position_nwu, const GeodeticPosition& origin);

/**
 * @brief Build IMU truth at the mounted point in sensor axes.
 *
 * The lever-arm terms alpha x r + omega x (omega x r) use only the true
 * angular acceleration and velocity from motion. The body-axis vibration
 * acceleration is then added to the point acceleration, and the body-axis
 * vibration angular velocity is added to the true gyro rate. Both sums are
 * formed before the mount orientation is applied.
 */
ImuTruth imu_truth(
    std::uint64_t sim_time_ns,
    const RigidBodyMotion& motion,
    const MountPose& mount,
    const World& world,
    const ImuVibration& vibration = {});

/// Build the runtime-convention earth field in NWU and in sensor axes.
MagTruth mag_truth(
    std::uint64_t sim_time_ns,
    const RigidBodyMotion& motion,
    const MountPose& mount,
    const World& world);

/// Build barometer truth at the mounted point; the atmosphere model remains
/// the adapter's responsibility.
BaroTruth baro_truth(
    std::uint64_t sim_time_ns,
    const RigidBodyMotion& motion,
    const MountPose& mount,
    const World& world);

/// Build antenna position and velocity in NWU and the matching WGS-84 fix.
GpsTruth gps_truth(
    std::uint64_t sim_time_ns,
    const RigidBodyMotion& motion,
    const MountPose& mount,
    const World& world);

}  // namespace hakoniwa::sensor_adapter::host
