cmake_minimum_required(VERSION 3.16)

project(hakoniwa_sensor_adapter LANGUAGES CXX)

add_library(hakoniwa_sensor_adapter INTERFACE)

target_include_directories(hakoniwa_sensor_adapter
    INTERFACE
        ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_compile_features(hakoniwa_sensor_adapter
    INTERFACE
        cxx_std_17
)

option(
    HAKONIWA_SENSOR_ADAPTER_BUILD_TESTS
    "Build standalone sensor-adapter contract tests"
    OFF
)

if(HAKONIWA_SENSOR_ADAPTER_BUILD_TESTS)
    enable_testing()

    add_executable(imu_sensor_adapter_contract_test test/imu_sensor_adapter_contract.cpp)
    target_link_libraries(imu_sensor_adapter_contract_test PRIVATE hakoniwa_sensor_adapter)
    add_test(NAME imu_sensor_adapter_contract COMMAND imu_sensor_adapter_contract_test)

    add_executable(mag_baro_gps_adapter_contract_test test/mag_baro_gps_adapter_contract.cpp)
    target_link_libraries(mag_baro_gps_adapter_contract_test PRIVATE hakoniwa_sensor_adapter)
    add_test(NAME mag_baro_gps_adapter_contract COMMAND mag_baro_gps_adapter_contract_test)

    add_executable(range_image_adapter_contract_test test/range_image_adapter_contract.cpp)
    target_link_libraries(range_image_adapter_contract_test PRIVATE hakoniwa_sensor_adapter)
    add_test(NAME range_image_adapter_contract COMMAND range_image_adapter_contract_test)

    # The external sensor plugin interface is C: a fake plugin in C, loaded by a C++ host.
    enable_language(C)
    add_library(fake_external_sensor_plugin MODULE test/external_sensor_plugin/fake_plugin.c)
    target_link_libraries(fake_external_sensor_plugin PRIVATE hakoniwa_sensor_adapter)
    set_target_properties(fake_external_sensor_plugin PROPERTIES C_VISIBILITY_PRESET hidden)
    add_executable(external_sensor_plugin_contract_test test/external_sensor_plugin_contract.cpp)
    target_link_libraries(external_sensor_plugin_contract_test PRIVATE hakoniwa_sensor_adapter ${CMAKE_DL_LIBS})
    target_compile_definitions(external_sensor_plugin_contract_test
        PRIVATE HAKO_FAKE_PLUGIN_PATH="$<TARGET_FILE:fake_external_sensor_plugin>")
    add_dependencies(external_sensor_plugin_contract_test fake_external_sensor_plugin)
    add_test(NAME external_sensor_plugin_contract COMMAND external_sensor_plugin_contract_test)
endif()
