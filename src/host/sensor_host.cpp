#include "hakoniwa/sensor_adapter/host/sensor_host.hpp"

#include <cmath>

namespace hakoniwa::sensor_adapter::host {
namespace {

constexpr double kPi = 3.141592653589793238462643383279502884;
constexpr double kDegToRad = kPi / 180.0;
constexpr double kRadToDeg = 180.0 / kPi;

// WGS-84 defining constants.
constexpr double kWgs84SemiMajorAxisM = 6378137.0;
constexpr double kWgs84Flattening = 1.0 / 298.257223563;
constexpr double kWgs84SemiMinorAxisM =
    kWgs84SemiMajorAxisM * (1.0 - kWgs84Flattening);
constexpr double kWgs84EccentricitySquared =
    kWgs84Flattening * (2.0 - kWgs84Flattening);
constexpr double kWgs84SecondEccentricitySquared =
    (kWgs84SemiMajorAxisM * kWgs84SemiMajorAxisM
        - kWgs84SemiMinorAxisM * kWgs84SemiMinorAxisM)
    / (kWgs84SemiMinorAxisM * kWgs84SemiMinorAxisM);

Vector3 add(const Vector3& a, const Vector3& b)
{
    return {a.x + b.x, a.y + b.y, a.z + b.z};
}

Vector3 subtract(const Vector3& a, const Vector3& b)
{
    return {a.x - b.x, a.y - b.y, a.z - b.z};
}

Vector3 cross(const Vector3& a, const Vector3& b)
{
    return {
        a.y * b.z - a.z * b.y,
        a.z * b.x - a.x * b.z,
        a.x * b.y - a.y * b.x};
}

Quaternion normalized(const Quaternion& q)
{
    const double norm = std::sqrt(q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z);
    if (norm == 0.0) {
        return {};
    }
    return {q.w / norm, q.x / norm, q.y / norm, q.z / norm};
}

Quaternion conjugate(const Quaternion& q)
{
    return {q.w, -q.x, -q.y, -q.z};
}

Quaternion multiply(const Quaternion& a, const Quaternion& b)
{
    return {
        a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z,
        a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
        a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
        a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w};
}

Vector3 rotate(const Quaternion& quaternion, const Vector3& vector)
{
    const Quaternion q = normalized(quaternion);
    const Vector3 qv{q.x, q.y, q.z};
    const Vector3 twice_cross{
        2.0 * (qv.y * vector.z - qv.z * vector.y),
        2.0 * (qv.z * vector.x - qv.x * vector.z),
        2.0 * (qv.x * vector.y - qv.y * vector.x)};
    return add(vector, add(
        {q.w * twice_cross.x, q.w * twice_cross.y, q.w * twice_cross.z},
        cross(qv, twice_cross)));
}

struct SensorPose {
    Vector3 position_nwu;
    Vector3 offset_nwu;
    Quaternion orientation_world_to_sensor;
};

SensorPose sensor_pose(const RigidBodyMotion& motion, const MountPose& mount)
{
    const Quaternion world_to_body = normalized(motion.orientation_world_to_body);
    const Quaternion body_to_sensor = normalized(mount.orientation_body_to_sensor);
    const Vector3 offset_nwu = rotate(conjugate(world_to_body), mount.position_body_m);
    return {
        add(motion.position_nwu, offset_nwu),
        offset_nwu,
        normalized(multiply(body_to_sensor, world_to_body))};
}

}  // namespace

MountPose mount_from_rpy_deg(
    const Vector3& position_body_m, double roll_deg, double pitch_deg, double yaw_deg)
{
    const double half_roll = 0.5 * roll_deg * kDegToRad;
    const double half_pitch = 0.5 * pitch_deg * kDegToRad;
    const double half_yaw = 0.5 * yaw_deg * kDegToRad;
    const Quaternion roll{std::cos(half_roll), std::sin(half_roll), 0.0, 0.0};
    const Quaternion pitch{std::cos(half_pitch), 0.0, std::sin(half_pitch), 0.0};
    const Quaternion yaw{std::cos(half_yaw), 0.0, 0.0, std::sin(half_yaw)};
    // Rz * Ry * Rx maps sensor coordinates into body coordinates.
    const Quaternion sensor_to_body = multiply(multiply(yaw, pitch), roll);
    return {position_body_m, normalized(conjugate(sensor_to_body))};
}

MountPose mount_from_frd_rpy_deg(
    const Vector3& position_frd_m, double roll_deg, double pitch_deg, double yaw_deg)
{
    // With C = diag(1, -1, -1), R_flu = C * R_frd * C. For
    // R_frd = Rz(yaw) * Ry(pitch) * Rx(roll), this is
    // Rz(-yaw) * Ry(-pitch) * Rx(roll).
    const Vector3 position_flu_m{
        position_frd_m.x, -position_frd_m.y, -position_frd_m.z};
    return mount_from_rpy_deg(
        position_flu_m, roll_deg, -pitch_deg, -yaw_deg);
}

GeodeticPosition nwu_to_geodetic(
    const Vector3& position_nwu, const GeodeticPosition& origin)
{
    const double latitude0 = origin.latitude_deg * kDegToRad;
    const double longitude0 = origin.longitude_deg * kDegToRad;
    const double sin_latitude0 = std::sin(latitude0);
    const double cos_latitude0 = std::cos(latitude0);
    const double sin_longitude0 = std::sin(longitude0);
    const double cos_longitude0 = std::cos(longitude0);
    const double prime_vertical_radius = kWgs84SemiMajorAxisM
        / std::sqrt(1.0 - kWgs84EccentricitySquared * sin_latitude0 * sin_latitude0);

    const double x0 = (prime_vertical_radius + origin.altitude_m)
        * cos_latitude0 * cos_longitude0;
    const double y0 = (prime_vertical_radius + origin.altitude_m)
        * cos_latitude0 * sin_longitude0;
    const double z0 = (prime_vertical_radius * (1.0 - kWgs84EccentricitySquared)
        + origin.altitude_m) * sin_latitude0;

    // NWU -> ENU, followed by the standard local-tangent ENU -> ECEF rotation.
    const double east = -position_nwu.y;
    const double north = position_nwu.x;
    const double up = position_nwu.z;
    const double x = x0 - sin_longitude0 * east
        - sin_latitude0 * cos_longitude0 * north
        + cos_latitude0 * cos_longitude0 * up;
    const double y = y0 + cos_longitude0 * east
        - sin_latitude0 * sin_longitude0 * north
        + cos_latitude0 * sin_longitude0 * up;
    const double z = z0 + cos_latitude0 * north + sin_latitude0 * up;

    const double longitude = std::atan2(y, x);
    const double distance_from_axis = std::hypot(x, y);
    if (distance_from_axis < 1e-12) {
        const double latitude = std::copysign(0.5 * kPi, z);
        return {
            latitude * kRadToDeg,
            longitude * kRadToDeg,
            std::abs(z) - kWgs84SemiMinorAxisM};
    }

    // Bowring's closed-form ECEF-to-geodetic solution.
    const double auxiliary = std::atan2(
        z * kWgs84SemiMajorAxisM,
        distance_from_axis * kWgs84SemiMinorAxisM);
    const double sin_auxiliary = std::sin(auxiliary);
    const double cos_auxiliary = std::cos(auxiliary);
    const double latitude = std::atan2(
        z + kWgs84SecondEccentricitySquared * kWgs84SemiMinorAxisM
            * sin_auxiliary * sin_auxiliary * sin_auxiliary,
        distance_from_axis - kWgs84EccentricitySquared * kWgs84SemiMajorAxisM
            * cos_auxiliary * cos_auxiliary * cos_auxiliary);
    const double sin_latitude = std::sin(latitude);
    const double radius = kWgs84SemiMajorAxisM
        / std::sqrt(1.0 - kWgs84EccentricitySquared * sin_latitude * sin_latitude);
    const double altitude = distance_from_axis / std::cos(latitude) - radius;
    return {latitude * kRadToDeg, longitude * kRadToDeg, altitude};
}

ImuTruth imu_truth(
    std::uint64_t sim_time_ns,
    const RigidBodyMotion& motion,
    const MountPose& mount,
    const World& world,
    const ImuVibration& vibration)
{
    const SensorPose pose = sensor_pose(motion, mount);
    const Quaternion world_to_body = normalized(motion.orientation_world_to_body);
    const Vector3 rotational_acceleration_body = add(
        cross(motion.angular_acceleration_body, mount.position_body_m),
        cross(motion.angular_velocity_body,
            cross(motion.angular_velocity_body, mount.position_body_m)));
    const Vector3 rotational_acceleration_nwu = rotate(
        conjugate(world_to_body),
        rotational_acceleration_body);
    const Vector3 true_point_acceleration_nwu = add(
        motion.linear_acceleration_nwu, rotational_acceleration_nwu);
    const Vector3 vibration_acceleration_nwu = rotate(
        conjugate(world_to_body), vibration.acceleration_body);
    const Vector3 observed_point_acceleration_nwu = add(
        true_point_acceleration_nwu, vibration_acceleration_nwu);
    const Vector3 observed_angular_velocity_body = add(
        motion.angular_velocity_body, vibration.angular_velocity_body);

    ImuTruth truth;
    truth.sim_time_ns = sim_time_ns;
    truth.angular_velocity = rotate(
        normalized(mount.orientation_body_to_sensor), observed_angular_velocity_body);
    truth.specific_force = rotate(
        pose.orientation_world_to_sensor,
        subtract(observed_point_acceleration_nwu, world.gravity_nwu));
    truth.orientation_world_to_sensor = pose.orientation_world_to_sensor;
    truth.gravity_world = world.gravity_nwu;
    return truth;
}

MagTruth mag_truth(
    std::uint64_t sim_time_ns,
    const RigidBodyMotion& motion,
    const MountPose& mount,
    const World& world)
{
    const SensorPose pose = sensor_pose(motion, mount);
    const double declination = world.geomagnetic_field.declination_deg * kDegToRad;
    const double inclination = world.geomagnetic_field.inclination_deg * kDegToRad;
    const double intensity = world.geomagnetic_field.intensity_nT;
    const Vector3 field_world{
        intensity * std::cos(inclination) * std::cos(declination),
        -intensity * std::cos(inclination) * std::sin(declination),
        -intensity * std::sin(inclination)};

    MagTruth truth;
    truth.sim_time_ns = sim_time_ns;
    truth.magnetic_field = rotate(pose.orientation_world_to_sensor, field_world);
    truth.magnetic_field_world = field_world;
    truth.orientation_world_to_sensor = pose.orientation_world_to_sensor;
    return truth;
}

BaroTruth baro_truth(
    std::uint64_t sim_time_ns,
    const RigidBodyMotion& motion,
    const MountPose& mount,
    const World& world)
{
    const SensorPose pose = sensor_pose(motion, mount);
    BaroTruth truth;
    truth.sim_time_ns = sim_time_ns;
    truth.altitude_m = world.origin.altitude_m - world.geoid_height_m
        + pose.position_nwu.z;
    truth.sea_level_pressure_pa = world.sea_level_pressure_pa;
    truth.sea_level_temperature_c = world.sea_level_temperature_c;
    return truth;
}

GpsTruth gps_truth(
    std::uint64_t sim_time_ns,
    const RigidBodyMotion& motion,
    const MountPose& mount,
    const World& world)
{
    const SensorPose pose = sensor_pose(motion, mount);
    const Vector3 angular_velocity_nwu = rotate(
        conjugate(normalized(motion.orientation_world_to_body)),
        motion.angular_velocity_body);

    GpsTruth truth;
    truth.sim_time_ns = sim_time_ns;
    truth.position_nwu = pose.position_nwu;
    truth.velocity_nwu = add(
        motion.velocity_nwu, cross(angular_velocity_nwu, pose.offset_nwu));
    truth.position = nwu_to_geodetic(pose.position_nwu, world.origin);
    return truth;
}

}  // namespace hakoniwa::sensor_adapter::host
