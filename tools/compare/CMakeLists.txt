# The drone sensor comparisons (docs/evaluation/drone): Hakoniwa, Gazebo and Project AirSim behind the same
# interface, fed the same truth. See README.md for the environment (pixi.toml) and the commands.
#
#   cmake -S tools/compare -B build-compare -G Ninja -DCMAKE_BUILD_TYPE=Release \
#     -DCMAKE_PREFIX_PATH=$CONDA_PREFIX -DHAKONIWA_SENSOR_PRO_DIR=<extracted hakoniwa-sensor-pro archive> \
#     [-DCOMPARE_WITH_AIRSIM=ON]
#
# The Hakoniwa implementation is reached by its name through the common interface (hakoniwa_entry.hpp); its code is
# the libraries of a hakoniwa-sensor-pro release archive. The Gazebo and Project AirSim adapters are built from
# their public checkouts next to this repository.
cmake_minimum_required(VERSION 3.16)
project(hakoniwa_sensor_compare LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

get_filename_component(_adapter_root "${CMAKE_CURRENT_SOURCE_DIR}/../.." ABSOLUTE)
get_filename_component(_workspace "${_adapter_root}/.." ABSOLUTE)

set(HAKONIWA_SENSOR_PRO_DIR "" CACHE PATH "An extracted hakoniwa-sensor-pro release archive")
set(HAKONIWA_SENSOR_ADAPTER_GAZEBO_ROOT "${_workspace}/hakoniwa-sensor-adapter-gazebo"
    CACHE PATH "hakoniwa-sensor-adapter-gazebo checkout")
set(HAKONIWA_SENSOR_ADAPTER_AIRSIM_ROOT "${_workspace}/hakoniwa-sensor-adapter-airsim"
    CACHE PATH "hakoniwa-sensor-adapter-airsim checkout")
option(COMPARE_WITH_AIRSIM "Add Project AirSim (core_sim) to the comparisons" OFF)

if(NOT EXISTS "${HAKONIWA_SENSOR_PRO_DIR}/lib/libhakoniwa_imu_gnss_sensor_models.a")
    message(FATAL_ERROR "HAKONIWA_SENSOR_PRO_DIR must point to an extracted hakoniwa-sensor-pro archive")
endif()
# The "hakoniwa" entry point and the IMU/GNSS adapters and models; they need no MuJoCo.
add_library(hakoniwa_sensor_pro INTERFACE)
target_link_libraries(hakoniwa_sensor_pro INTERFACE
    "${HAKONIWA_SENSOR_PRO_DIR}/lib/libhakoniwa_mujoco_sensors_adapters.a"
    "${HAKONIWA_SENSOR_PRO_DIR}/lib/libhakoniwa_imu_gnss_sensor_adapters.a"
    "${HAKONIWA_SENSOR_PRO_DIR}/lib/libhakoniwa_imu_gnss_sensor_models.a")

# The Gazebo adapters (gz-sensors) bring the interface of this repository with them.
set(HAKONIWA_SENSOR_ADAPTER_ROOT "${_adapter_root}" CACHE PATH "" FORCE)
add_subdirectory(${HAKONIWA_SENSOR_ADAPTER_GAZEBO_ROOT} hakoniwa-sensor-adapter-gazebo)
# Axis 0 converts the local position to latitude and longitude with gz-math, as gz-sim does for NavSat.
find_package(gz-math REQUIRED)

if(COMPARE_WITH_AIRSIM)
    # Project AirSim's own core_sim sensors, compiled unchanged from the sources its lock file pins.
    set(HAKONIWA_SENSOR_ADAPTER_AIRSIM_NATIVE ON CACHE BOOL "" FORCE)
    add_subdirectory(${HAKONIWA_SENSOR_ADAPTER_AIRSIM_ROOT} hakoniwa-sensor-adapter-airsim)
endif()

foreach(_axis axis0_no_noise axis1_accuracy axis3_repeatability axis4_speed)
    add_executable(compare_${_axis} ${_axis}.cpp)
    target_include_directories(compare_${_axis} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
    target_link_libraries(compare_${_axis} PRIVATE hakoniwa_sensor_pro hakoniwa_sensor_adapter_gazebo)
    if(COMPARE_WITH_AIRSIM)
        target_link_libraries(compare_${_axis} PRIVATE hakoniwa_sensor_adapter_airsim_native)
        target_compile_definitions(compare_${_axis} PRIVATE HAKO_SENSORS_WITH_AIRSIM)
    endif()
endforeach()
target_link_libraries(compare_axis0_no_noise PRIVATE gz-math::gz-math)
target_compile_definitions(compare_axis0_no_noise PRIVATE HAKO_SENSORS_CONFIG_DIR="${CMAKE_CURRENT_SOURCE_DIR}/config")

# The force-torque comparison (docs/evaluation/force-torque): Hakoniwa and Gazebo; Project AirSim has none.
add_executable(compare_force_torque force_torque.cpp)
target_include_directories(compare_force_torque PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(compare_force_torque PRIVATE hakoniwa_sensor_pro hakoniwa_sensor_adapter_gazebo)
target_compile_definitions(compare_force_torque
    PRIVATE HAKO_FT_COMPARE_CONFIG_DIR="${CMAKE_CURRENT_SOURCE_DIR}/config/force-torque")

# Time series of one force-torque sensor, for the measurement cards of the force-torque page.
add_executable(ft-series ft_series.cpp)
target_include_directories(ft-series PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(ft-series PRIVATE hakoniwa_sensor_pro hakoniwa_sensor_adapter_gazebo)
