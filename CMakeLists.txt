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

# The reference implementation (true values only; see
# include/hakoniwa/sensor_adapter/reference/). Optional: it needs nlohmann/json
# (header-only), found as a package or given as a directory. The interface
# library above needs nothing.
set(HAKONIWA_SENSOR_ADAPTER_NLOHMANN_JSON_INCLUDE_DIR "" CACHE PATH
    "Directory that contains nlohmann/json.hpp (when nlohmann_json is not installed as a package)")
find_package(nlohmann_json 3 QUIET)
if(nlohmann_json_FOUND OR EXISTS "${HAKONIWA_SENSOR_ADAPTER_NLOHMANN_JSON_INCLUDE_DIR}/nlohmann/json.hpp")
    add_library(hakoniwa_sensor_adapter_reference STATIC src/reference/reference_implementation.cpp)
    target_link_libraries(hakoniwa_sensor_adapter_reference PUBLIC hakoniwa_sensor_adapter)
    if(nlohmann_json_FOUND)
        target_link_libraries(hakoniwa_sensor_adapter_reference PUBLIC nlohmann_json::nlohmann_json)
    else()
        target_include_directories(hakoniwa_sensor_adapter_reference PUBLIC
            ${HAKONIWA_SENSOR_ADAPTER_NLOHMANN_JSON_INCLUDE_DIR})
    endif()
    set_target_properties(hakoniwa_sensor_adapter_reference PROPERTIES POSITION_INDEPENDENT_CODE ON)
endif()

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

    add_executable(force_torque_adapter_contract_test test/force_torque_adapter_contract.cpp)
    target_link_libraries(force_torque_adapter_contract_test PRIVATE hakoniwa_sensor_adapter)
    add_test(NAME force_torque_adapter_contract COMMAND force_torque_adapter_contract_test)

    add_executable(joint_sensor_adapter_contract_test test/joint_sensor_adapter_contract.cpp)
    target_link_libraries(joint_sensor_adapter_contract_test PRIVATE hakoniwa_sensor_adapter)
    add_test(NAME joint_sensor_adapter_contract COMMAND joint_sensor_adapter_contract_test)

    add_executable(sensor_adapter_implementation_contract_test test/sensor_adapter_implementation_contract.cpp)
    target_link_libraries(sensor_adapter_implementation_contract_test PRIVATE hakoniwa_sensor_adapter)
    add_test(NAME sensor_adapter_implementation_contract COMMAND sensor_adapter_implementation_contract_test)

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

    if(TARGET hakoniwa_sensor_adapter_reference)
        add_executable(reference_implementation_test test/reference_implementation_test.cpp)
        target_link_libraries(reference_implementation_test PRIVATE hakoniwa_sensor_adapter_reference)
        target_compile_definitions(reference_implementation_test
            PRIVATE REFERENCE_TEST_DIR="${CMAKE_CURRENT_BINARY_DIR}")
        add_test(NAME reference_implementation COMMAND reference_implementation_test)
    else()
        message(STATUS "nlohmann/json not found: the reference implementation and its test are skipped")
    endif()

    # Sensor definition schema: examples and docs/spec examples (needs Python with jsonschema).
    find_package(Python3 COMPONENTS Interpreter)
    if(Python3_Interpreter_FOUND)
        execute_process(COMMAND ${Python3_EXECUTABLE} -c "import jsonschema"
            RESULT_VARIABLE _hako_jsonschema_missing OUTPUT_QUIET ERROR_QUIET)
        if(_hako_jsonschema_missing EQUAL 0)
            add_test(NAME sensor_definition_schema
                COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/tools/check_sensor_definition.py --self-test)
        else()
            message(STATUS "jsonschema not found: the sensor definition schema test is skipped")
        endif()
    endif()
endif()
