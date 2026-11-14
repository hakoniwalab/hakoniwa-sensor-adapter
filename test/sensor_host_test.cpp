#include "hakoniwa/sensor_adapter/host/sensor_host.hpp"

#include <cmath>
#include <cstdio>
#include <cstdlib>

namespace host = hakoniwa::sensor_adapter::host;
namespace sa = hakoniwa::sensor_adapter;

namespace {

constexpr double kPi = 3.141592653589793238462643383279502884;
constexpr double kDegToRad = kPi / 180.0;
int failures = 0;

void check(bool condition, const char* what)
{
    if (!condition) {
        std::fprintf(stderr, "FAIL: %s\n", what);
        ++failures;
    }
}

void check_near(double actual, double expected, double tolerance, const char* what)
{
    if (std::abs(actual - expected) > tolerance) {
        std::fprintf(stderr, "FAIL: %s: got %.15g, expected %.15g (+/- %.3g)\n",
            what, actual, expected, tolerance);
        ++failures;
    }
}

void check_vector(
    const sa::Vector3& actual, const sa::Vector3& expected, double tolerance, const char* what)
{
    if (std::abs(actual.x - expected.x) > tolerance
        || std::abs(actual.y - expected.y) > tolerance
        || std::abs(actual.z - expected.z) > tolerance) {
        std::fprintf(stderr,
            "FAIL: %s: got {%.15g, %.15g, %.15g}, expected {%.15g, %.15g, %.15g}\n",
            what, actual.x, actual.y, actual.z, expected.x, expected.y, expected.z);
        ++failures;
    }
}

struct Matrix3 {
    double value[3][3]{};
};

void check_matrix(
    const Matrix3& actual, const Matrix3& expected, double tolerance, const char* what)
{
    for (int row = 0; row < 3; ++row) {
        for (int column = 0; column < 3; ++column) {
            if (std::abs(actual.value[row][column] - expected.value[row][column]) > tolerance) {
                std::fprintf(stderr,
                    "FAIL: %s[%d][%d]: got %.15g, expected %.15g (+/- %.3g)\n",
                    what, row, column, actual.value[row][column],
                    expected.value[row][column], tolerance);
                ++failures;
            }
        }
    }
}

Matrix3 multiply(const Matrix3& left, const Matrix3& right)
{
    Matrix3 result{};
    for (int row = 0; row < 3; ++row) {
        for (int column = 0; column < 3; ++column) {
            for (int term = 0; term < 3; ++term) {
                result.value[row][column] +=
                    left.value[row][term] * right.value[term][column];
            }
        }
    }
    return result;
}

Matrix3 transpose(const Matrix3& matrix)
{
    Matrix3 result{};
    for (int row = 0; row < 3; ++row) {
        for (int column = 0; column < 3; ++column) {
            result.value[row][column] = matrix.value[column][row];
        }
    }
    return result;
}

Matrix3 rotation_x(double angle_deg)
{
    const double angle = angle_deg * kDegToRad;
    const double cosine = std::cos(angle);
    const double sine = std::sin(angle);
    return {{{1.0, 0.0, 0.0}, {0.0, cosine, -sine}, {0.0, sine, cosine}}};
}

Matrix3 rotation_y(double angle_deg)
{
    const double angle = angle_deg * kDegToRad;
    const double cosine = std::cos(angle);
    const double sine = std::sin(angle);
    return {{{cosine, 0.0, sine}, {0.0, 1.0, 0.0}, {-sine, 0.0, cosine}}};
}

Matrix3 rotation_z(double angle_deg)
{
    const double angle = angle_deg * kDegToRad;
    const double cosine = std::cos(angle);
    const double sine = std::sin(angle);
    return {{{cosine, -sine, 0.0}, {sine, cosine, 0.0}, {0.0, 0.0, 1.0}}};
}

Matrix3 matrix_from_quaternion(const sa::Quaternion& q)
{
    const double norm = std::sqrt(q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z);
    const double w = q.w / norm;
    const double x = q.x / norm;
    const double y = q.y / norm;
    const double z = q.z / norm;
    return {{{
        1.0 - 2.0 * (y * y + z * z),
        2.0 * (x * y - z * w),
        2.0 * (x * z + y * w)}, {
        2.0 * (x * y + z * w),
        1.0 - 2.0 * (x * x + z * z),
        2.0 * (y * z - x * w)}, {
        2.0 * (x * z - y * w),
        2.0 * (y * z + x * w),
        1.0 - 2.0 * (x * x + y * y)}}};
}

// An independent inverse used only to assess the production NWU -> geodetic
// conversion. These are the standard WGS-84 geodetic -> ECEF equations,
// followed by the ECEF -> ENU matrix. WGS-84 defines a = 6378137 m and
// f = 1/298.257223563.
sa::Vector3 geodetic_to_nwu_for_test(
    const sa::GeodeticPosition& position, const sa::GeodeticPosition& origin)
{
    constexpr double a = 6378137.0;
    constexpr double f = 1.0 / 298.257223563;
    constexpr double e2 = f * (2.0 - f);
    auto to_ecef = [](const sa::GeodeticPosition& geodetic, double out[3]) {
        const double latitude = geodetic.latitude_deg * kDegToRad;
        const double longitude = geodetic.longitude_deg * kDegToRad;
        const double sin_latitude = std::sin(latitude);
        const double n = a / std::sqrt(1.0 - e2 * sin_latitude * sin_latitude);
        out[0] = (n + geodetic.altitude_m) * std::cos(latitude) * std::cos(longitude);
        out[1] = (n + geodetic.altitude_m) * std::cos(latitude) * std::sin(longitude);
        out[2] = (n * (1.0 - e2) + geodetic.altitude_m) * sin_latitude;
    };

    double point_ecef[3];
    double origin_ecef[3];
    to_ecef(position, point_ecef);
    to_ecef(origin, origin_ecef);
    const double dx = point_ecef[0] - origin_ecef[0];
    const double dy = point_ecef[1] - origin_ecef[1];
    const double dz = point_ecef[2] - origin_ecef[2];
    const double latitude = origin.latitude_deg * kDegToRad;
    const double longitude = origin.longitude_deg * kDegToRad;
    const double east = -std::sin(longitude) * dx + std::cos(longitude) * dy;
    const double north = -std::sin(latitude) * std::cos(longitude) * dx
        - std::sin(latitude) * std::sin(longitude) * dy + std::cos(latitude) * dz;
    const double up = std::cos(latitude) * std::cos(longitude) * dx
        + std::cos(latitude) * std::sin(longitude) * dy + std::sin(latitude) * dz;
    return {north, -east, up};
}

void test_imu_at_rest()
{
    const host::RigidBodyMotion motion{};
    const host::MountPose mount{};
    const host::World world{};
    const sa::ImuTruth truth = host::imu_truth(123, motion, mount, world);

    // At rest a_kinematic = 0. Specific force is 0 - (0,0,-9.80665), so the
    // hand-worked FLU result is (0,0,+9.80665). There is no angular motion.
    check_vector(truth.specific_force, {0.0, 0.0, 9.80665}, 1e-12,
        "rest: specific force is +g on sensor z");
    check_vector(truth.angular_velocity, {}, 1e-12, "rest: zero angular velocity");
    check(truth.sim_time_ns == 123, "rest: simulation time is copied");
    check_vector(truth.gravity_world, {0.0, 0.0, -9.80665}, 0.0,
        "rest: world gravity is copied");
}

void test_imu_rigid_body_acceleration()
{
    host::RigidBodyMotion motion{};
    motion.angular_velocity_body = {0.0, 0.0, 2.0};
    motion.angular_acceleration_body = {0.0, 0.0, 5.0};
    const host::MountPose mount = host::mount_from_rpy_deg({3.0, 0.0, 0.0}, 0.0, 0.0, 0.0);
    const sa::ImuTruth truth = host::imu_truth(0, motion, mount, {});

    // By hand, omega x (omega x (3,0,0)) = (-2^2*3,0,0) = (-12,0,0),
    // and alpha x r = (0,5*3,0) = (0,15,0). Subtracting downward gravity
    // adds +9.80665 on z.
    check_vector(truth.specific_force, {-12.0, 15.0, 9.80665}, 1e-12,
        "spin: centripetal and tangential specific force");
    check_vector(truth.angular_velocity, {0.0, 0.0, 2.0}, 1e-12,
        "spin: angular velocity in aligned sensor axes");

    host::RigidBodyMotion translating{};
    translating.linear_acceleration_nwu = {4.25, 0.0, 0.0};
    const sa::ImuTruth translating_truth = host::imu_truth(0, translating, {}, {});
    // Level FLU equals NWU, so a hand-selected 4.25 m/s^2 north acceleration
    // remains +4.25 on sensor x; only the z component receives +g.
    check_vector(translating_truth.specific_force, {4.25, 0.0, 9.80665}, 1e-12,
        "translation: world x acceleration is sensor x acceleration");
}

void test_imu_vibration_order()
{
    host::RigidBodyMotion motion{};
    motion.angular_velocity_body = {0.0, 0.0, 2.0};
    const host::MountPose mount =
        host::mount_from_rpy_deg({3.0, 0.0, 0.0}, 0.0, 0.0, 90.0);
    const sa::ImuTruth without_vibration = host::imu_truth(0, motion, mount, {});
    const host::ImuVibration vibration{{4.0, 5.0, 6.0}, {1.5, 0.0, 0.0}};
    const sa::ImuTruth with_vibration =
        host::imu_truth(0, motion, mount, {}, vibration);

    // True omega alone gives omega x (omega x r) = (-12,0,0). A +90 degree
    // mount yaw maps body (x,y,z) to sensor (y,-x,z), so without vibration
    // (-12,0,g) becomes (0,12,g).
    check_vector(without_vibration.specific_force, {0.0, 12.0, 9.80665}, 1e-12,
        "vibration: baseline centripetal term uses true omega");
    // Add body vibration acceleration (4,5,6) only after that lever-arm term:
    // (-12,0,0)+(4,5,6)-(0,0,-g)=(-8,5,15.80665), then the mount
    // rotation gives (5,8,15.80665). If vibration omega were incorrectly used
    // in the centripetal term, it would add a spurious +9 on body z.
    check_vector(with_vibration.specific_force, {5.0, 8.0, 15.80665}, 1e-12,
        "vibration: acceleration is added after true lever-arm kinematics");
    // The gyro sum is (1.5,0,2) in body FLU before the same mount rotation,
    // hence (0,-1.5,2) in sensor axes.
    check_vector(with_vibration.angular_velocity, {0.0, -1.5, 2.0}, 1e-12,
        "vibration: angular velocity is added before mount rotation");
}

void test_frd_mount_conversion()
{
    const host::MountPose yaw_mount =
        host::mount_from_frd_rpy_deg({1.0, 2.0, 3.0}, 0.0, 0.0, 90.0);
    check_vector(yaw_mount.position_body_m, {1.0, -2.0, -3.0}, 0.0,
        "FRD mount: position y and z are negated");
    const Matrix3 yaw_sensor_to_body =
        transpose(matrix_from_quaternion(yaw_mount.orientation_body_to_sensor));
    // FRD yaw +90 points sensor x along body right. Body right is FLU -y,
    // so the first column of R_body_from_sensor must be (0,-1,0).
    check_vector({
        yaw_sensor_to_body.value[0][0],
        yaw_sensor_to_body.value[1][0],
        yaw_sensor_to_body.value[2][0]},
        {0.0, -1.0, 0.0}, 1e-12,
        "FRD mount: yaw +90 sensor x points along FLU -y");

    const host::MountPose pitch_mount =
        host::mount_from_frd_rpy_deg({}, 0.0, 30.0, 0.0);
    const Matrix3 pitch_sensor_to_body =
        transpose(matrix_from_quaternion(pitch_mount.orientation_body_to_sensor));
    // C*Ry_FRD(+30)*C = Ry_FLU(-30). Its sensor-x column is
    // (cos30,0,+sin30) = (sqrt(3)/2,0,1/2) in body FLU.
    check_vector({
        pitch_sensor_to_body.value[0][0],
        pitch_sensor_to_body.value[1][0],
        pitch_sensor_to_body.value[2][0]},
        {std::sqrt(3.0) / 2.0, 0.0, 0.5}, 1e-12,
        "FRD mount: pitch +30 becomes FLU pitch -30");

    // Combined hand check for FRD roll=30, pitch=45, yaw=60. Build
    // R_frd=Rz(60)*Ry(45)*Rx(30), then conjugate it by
    // C=diag(1,-1,-1). The hard-coded matrices below come from multiplying
    // those three elementary matrices by hand.
    const Matrix3 rotation_frd = multiply(
        rotation_z(60.0), multiply(rotation_y(45.0), rotation_x(30.0)));
    const Matrix3 expected_frd{{
        {0.3535533905932738, -0.5732233047033631, 0.7391989197401165},
        {0.6123724356957945, 0.7391989197401165, 0.2803300858899106},
        {-0.7071067811865475, 0.3535533905932738, 0.6123724356957946}}};
    check_matrix(rotation_frd, expected_frd, 1e-12,
        "FRD mount: hand-built combined FRD matrix");

    const Matrix3 axis_flip{{
        {1.0, 0.0, 0.0}, {0.0, -1.0, 0.0}, {0.0, 0.0, -1.0}}};
    const Matrix3 conjugated_flu = multiply(axis_flip, multiply(rotation_frd, axis_flip));
    const Matrix3 signed_angle_flu = multiply(
        rotation_z(-60.0), multiply(rotation_y(-45.0), rotation_x(30.0)));
    const Matrix3 expected_flu{{
        {0.3535533905932738, 0.5732233047033631, -0.7391989197401165},
        {-0.6123724356957945, 0.7391989197401165, 0.2803300858899106},
        {0.7071067811865475, 0.3535533905932738, 0.6123724356957946}}};
    check_matrix(conjugated_flu, expected_flu, 1e-12,
        "FRD mount: C times R_frd times C");
    check_matrix(signed_angle_flu, expected_flu, 1e-12,
        "FRD mount: roll unchanged and pitch/yaw negated");

    const host::MountPose combined_mount =
        host::mount_from_frd_rpy_deg({}, 30.0, 45.0, 60.0);
    const Matrix3 helper_sensor_to_body =
        transpose(matrix_from_quaternion(combined_mount.orientation_body_to_sensor));
    check_matrix(helper_sensor_to_body, expected_flu, 1e-12,
        "FRD mount: helper matches hand-built combined FLU matrix");
}

void test_rotated_mounts()
{
    host::RigidBodyMotion yaw_motion{};
    yaw_motion.linear_acceleration_nwu = {2.0, 0.0, 0.0};
    yaw_motion.angular_velocity_body = {3.0, 0.0, 0.0};
    const host::MountPose yaw_mount = host::mount_from_rpy_deg({}, 0.0, 0.0, 90.0);
    const sa::ImuTruth yaw_truth = host::imu_truth(0, yaw_motion, yaw_mount, {});

    // A +90 degree mount yaw points sensor x along body +y. Consequently body
    // +x has sensor coordinates -y. By inspection, (2,0,g) -> (0,-2,g) and
    // gyro (3,0,0) -> (0,-3,0). The world->sensor quaternion is yaw -90:
    // (cos 45,0,0,-sin 45).
    check_vector(yaw_truth.specific_force, {0.0, -2.0, 9.80665}, 1e-12,
        "yaw mount: specific force rotates into sensor axes");
    check_vector(yaw_truth.angular_velocity, {0.0, -3.0, 0.0}, 1e-12,
        "yaw mount: gyro rotates into sensor axes");
    const double sqrt_half = std::sqrt(0.5);
    check_near(yaw_truth.orientation_world_to_sensor.w, sqrt_half, 1e-12,
        "yaw mount: quaternion w");
    check_near(yaw_truth.orientation_world_to_sensor.z, -sqrt_half, 1e-12,
        "yaw mount: quaternion z");

    host::RigidBodyMotion roll_motion{};
    roll_motion.angular_velocity_body = {0.0, 2.0, 0.0};
    const host::MountPose roll_mount = host::mount_from_rpy_deg({}, 180.0, 0.0, 0.0);
    const sa::ImuTruth roll_truth = host::imu_truth(0, roll_motion, roll_mount, {});
    // A 180 degree roll keeps x and reverses y and z, hence rest +g on body z
    // becomes -g on sensor z and body gyro +y becomes sensor -y.
    check_vector(roll_truth.specific_force, {0.0, 0.0, -9.80665}, 1e-12,
        "roll mount: gravity reverses sensor z");
    check_vector(roll_truth.angular_velocity, {0.0, -2.0, 0.0}, 1e-12,
        "roll mount: sensor y is reversed");
    check_near(std::abs(roll_truth.orientation_world_to_sensor.x), 1.0, 1e-12,
        "roll mount: quaternion is a half turn about x");
}

void test_body_pose_and_antenna_velocity()
{
    host::RigidBodyMotion motion{};
    motion.position_nwu = {10.0, 20.0, 30.0};
    motion.velocity_nwu = {1.0, 2.0, 3.0};
    // Body->world is Rz(+90) Ry(+90). Half-angle multiplication gives
    // q_body_to_world=(0.5,-0.5,0.5,0.5); the stored inverse is below.
    motion.orientation_world_to_body = {0.5, 0.5, -0.5, -0.5};
    motion.angular_velocity_body = {0.0, 0.0, 2.0};
    const host::MountPose mount = host::mount_from_rpy_deg({2.0, 3.0, 4.0}, 0.0, 0.0, 0.0);
    const sa::GpsTruth truth = host::gps_truth(77, motion, mount, {});

    // The two hand-written rotations map body basis vectors as follows:
    // x_body -> -z_world, y_body -> -x_world, z_body -> +y_world.
    // Therefore r=(2,3,4) becomes (-3,4,-2), and p=(10,20,30)+r=(7,24,28).
    check_vector(truth.position_nwu, {7.0, 24.0, 28.0}, 1e-12,
        "pitched and yawed body: antenna position");
    // omega=(0,0,2)_body becomes (0,2,0)_world. By hand,
    // omega x r = (-4,0,6), which added to (1,2,3) gives (-3,2,9).
    check_vector(truth.velocity_nwu, {-3.0, 2.0, 9.0}, 1e-12,
        "pitched and yawed body: antenna velocity");
    check(truth.sim_time_ns == 77, "GPS: simulation time is copied");
}

void test_magnetic_field()
{
    host::World world{};
    world.geomagnetic_field = {50000.0, 0.0, 0.0};
    const sa::MagTruth north = host::mag_truth(0, {}, {}, world);
    // With zero declination and inclination, the full 50000 nT intensity is
    // horizontal north: NWU and level FLU both give (50000,0,0).
    check_vector(north.magnetic_field_world, {50000.0, 0.0, 0.0}, 1e-9,
        "magnetic field: horizontal north in NWU");
    check_vector(north.magnetic_field, {50000.0, 0.0, 0.0}, 1e-9,
        "magnetic field: horizontal north in level sensor");

    world.geomagnetic_field = {50000.0, 10.0, 60.0};
    const sa::MagTruth angled = host::mag_truth(0, {}, {}, world);
    // Hand calculation using positive-east declination and positive-down
    // inclination: north=50000*cos60*cos10=24620.1938253 nT,
    // west=-50000*cos60*sin10=-4341.20444167 nT, and
    // up=-50000*sin60=-43301.2701892 nT.
    const sa::Vector3 expected_world{
        24620.1938253052, -4341.20444167326, -43301.2701892219};
    check_vector(angled.magnetic_field_world, expected_world, 1e-8,
        "magnetic field: declination and inclination components");

    const host::MountPose yaw_mount = host::mount_from_rpy_deg({}, 0.0, 0.0, 90.0);
    const sa::MagTruth yawed = host::mag_truth(0, {}, yaw_mount, world);
    // The +90 mount yaw maps (north,west,up) to (west,-north,up), by inspection.
    check_vector(yawed.magnetic_field,
        {-4341.20444167326, -24620.1938253052, -43301.2701892219}, 1e-8,
        "magnetic field: yawed sensor components");
}

void test_barometer_altitude()
{
    host::RigidBodyMotion motion{};
    motion.position_nwu = {0.0, 0.0, 10.0};
    const host::MountPose mount = host::mount_from_rpy_deg({0.0, 0.0, 2.0}, 0.0, 0.0, 0.0);
    host::World world{};
    world.origin = {35.0, 139.0, 100.3};
    world.geoid_height_m = 30.5;
    world.sea_level_pressure_pa = 100800.0;
    world.sea_level_temperature_c = 12.5;
    const sa::BaroTruth truth = host::baro_truth(9, motion, mount, world);

    // The origin's MSL height is h-N = 100.3-30.5 = 69.8 m. The sensor is
    // 10+2=12 m above it, so its barometric altitude is 81.8 m MSL.
    check_near(truth.altitude_m, 81.8, 1e-12, "barometer: sensor-point MSL altitude");
    check_near(truth.sea_level_pressure_pa, 100800.0, 0.0,
        "barometer: sea-level pressure is copied");
    check_near(truth.sea_level_temperature_c, 12.5, 0.0,
        "barometer: sea-level temperature is copied");
    check(truth.sim_time_ns == 9, "barometer: simulation time is copied");
}

void test_geodetic_conversion()
{
    const sa::GeodeticPosition origin{35.0, 139.0, 50.0};
    const sa::GeodeticPosition same = host::nwu_to_geodetic({}, origin);
    // A zero local displacement is the origin by definition.
    check_near(same.latitude_deg, 35.0, 1e-12, "geodetic: origin latitude");
    check_near(same.longitude_deg, 139.0, 1e-12, "geodetic: origin longitude");
    check_near(same.altitude_m, 50.0, 1e-8, "geodetic: origin altitude");

    const sa::GeodeticPosition equator{};
    const sa::GeodeticPosition north = host::nwu_to_geodetic({1000.0, 0.0, 0.0}, equator);
    // Independent published implementation check: PROJ 9.5.1 transformation
    // EPSG:4978 -> EPSG:4979 maps ECEF (6378137,0,1000) m to longitude 0 deg,
    // latitude 0.009043694694644246 deg, height 0.07892112340778112 m.
    // The ECEF point follows directly because WGS-84 a=6378137 m and north at
    // latitude/longitude zero is +ECEF z.
    check_near(north.latitude_deg, 0.009043694694644246, 1e-13,
        "geodetic: published 1000 m north latitude");
    check_near(north.longitude_deg, 0.0, 1e-13,
        "geodetic: published 1000 m north longitude");
    check_near(north.altitude_m, 0.07892112340778112, 1e-8,
        "geodetic: published 1000 m north ellipsoid height");

    host::RigidBodyMotion motion{};
    motion.position_nwu = {1234.5, -678.9, 42.0};
    host::World world{};
    world.origin = origin;
    const sa::GpsTruth truth = host::gps_truth(0, motion, {}, world);
    const sa::Vector3 recovered = geodetic_to_nwu_for_test(truth.position, origin);
    // The expected local point is the hand-selected input. Recovery uses the
    // independent forward WGS-84 ECEF equations above, not the implementation's
    // Bowring ECEF->geodetic calculation. Each coordinate must be below 1 mm.
    check_vector(recovered, motion.position_nwu, 0.001,
        "geodetic: independent NWU round trip below 1 mm");
}

}  // namespace

int main()
{
    test_imu_at_rest();
    test_imu_rigid_body_acceleration();
    test_imu_vibration_order();
    test_rotated_mounts();
    test_frd_mount_conversion();
    test_body_pose_and_antenna_velocity();
    test_magnetic_field();
    test_barometer_altitude();
    test_geodetic_conversion();

    if (failures == 0) {
        std::printf("sensor_host_test: OK\n");
    }
    return failures == 0 ? EXIT_SUCCESS : EXIT_FAILURE;
}
