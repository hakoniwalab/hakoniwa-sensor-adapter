/*
 * External sensor plugin interface (C).
 *
 * A host simulator (for example Hakoniwa Drone PRO) loads a shared library that
 * implements range and image sensors measured in the host's physics world. The
 * plugin owns the sensor configuration, the measurement, the encoding and the
 * Hakoniwa PDU / service I/O; the host only loads it, passes the world and the
 * time, follows the lifecycle below, and lends its main thread when asked.
 *
 * Only C types cross this boundary: no C++ object, no standard-library type,
 * no exception, and no memory allocated on one side and freed on the other.
 * A host and a plugin built with different compilers, C runtimes (MSVC /MT and
 * /MD) or Debug/Release settings therefore work together.
 *
 * Lifecycle (one sensor instance; the host calls these in this order):
 *   create             once, while the simulation is stopped
 *   register_services  once, after the Hakoniwa asset is registered and before
 *                      the first start (the plugin may call
 *                      hako_asset_service_initialize with its own service file)
 *   on_start           at the start event, before the start feedback (the plugin
 *                      may create its service servers)
 *   step               every physics step while running, right after the physics
 *                      step, on the simulation thread
 *   on_stop            when the simulation stops (drop requests in progress)
 *   reset              when the simulation is reset (back to the state at create)
 *   destroy            once, at the end
 * main_thread_poll may be called repeatedly on the host's main thread at any time
 * between create and destroy. A sensor whose needs_main_thread returns 1 (for
 * example a camera on macOS) relies on it: its step may wait until the main
 * thread has done the work. While a step waits, the host does not advance the
 * physics, so the work sees the state of that step. on_stop and destroy release a
 * step that is waiting (it fails with an error instead of blocking). For such a
 * sensor the host calls destroy on the main thread too (the main-thread resources,
 * such as an OpenGL window on macOS, must be released there).
 *
 * Every function returns 0 on success. On failure it returns non-zero and writes a
 * NUL-terminated message into error (at most error_size bytes, truncated).
 */
#ifndef HAKONIWA_SENSOR_ADAPTER_EXTERNAL_SENSOR_PLUGIN_H
#define HAKONIWA_SENSOR_ADAPTER_EXTERNAL_SENSOR_PLUGIN_H

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define HAKO_EXTERNAL_SENSOR_PLUGIN_ABI_VERSION 1u
/* Changes whenever a struct or a function of this header changes. */
#define HAKO_EXTERNAL_SENSOR_PLUGIN_INTERFACE_REVISION "external-sensor-plugin/1"
#define HAKO_EXTERNAL_SENSOR_PLUGIN_ENTRY "hako_external_sensor_plugin_v1"

#if defined(_WIN32)
#define HAKO_EXTERNAL_SENSOR_PLUGIN_EXPORT __declspec(dllexport)
#else
#define HAKO_EXTERNAL_SENSOR_PLUGIN_EXPORT __attribute__((visibility("default")))
#endif

typedef struct HakoExternalSensorVector3 {
    double x, y, z;
} HakoExternalSensorVector3;

/* Unit quaternion (w, x, y, z). */
typedef struct HakoExternalSensorQuaternion {
    double w, x, y, z;
} HakoExternalSensorQuaternion;

/* What the host passes to create one sensor. */
typedef struct HakoExternalSensorCreateV1 {
    /* sizeof(HakoExternalSensorCreateV1) as the host built it. */
    uint32_t struct_size;

    /* The sensor: "rangefinder", "lidar", "depth_camera" or "rgb_camera", and the
     * plugin's own configuration file (the host does not read it). */
    const char* kind;
    const char* config_path;

    /* Names for logs, PDUs, services and random streams. */
    const char* vehicle_name;
    uint32_t vehicle_index;
    const char* sensor_name;

    /* Time and randomness. */
    uint64_t physics_step_ns;
    int32_t has_random_seed;
    uint64_t random_seed;

    /* How often the plugin writes its output PDUs [Hz]; 0: on request only. */
    double publish_rate_hz;

    /* The host's physics world, opaque to this header. For MuJoCo: world_engine
     * "mujoco", world_engine_version mj_version(), world_model a const mjModel*,
     * world_data an mjData*. The plugin reads them and never changes them. */
    const char* world_engine;
    int32_t world_engine_version;
    const void* world_model;
    void* world_data;

    /* Where the sensor is: the body that carries it (the engine's body id) and the
     * mounting pose relative to that body's frame, in that frame's axes
     * (for MuJoCo, the body frame of the model). Bodies under it are the vehicle:
     * the sensor never sees them. */
    int32_t body_id;
    HakoExternalSensorVector3 mount_position;
    HakoExternalSensorQuaternion mount_orientation;

    /* Hakoniwa context. service_config_dir is where the plugin writes the service
     * file it registers, so that client processes can read the same list. */
    const char* asset_name;
    const char* robot_name;
    const char* pdu_config_path;
    const char* service_config_dir;
} HakoExternalSensorCreateV1;

typedef struct HakoExternalSensorPluginV1 {
    uint32_t struct_size;               /* sizeof(HakoExternalSensorPluginV1) as the plugin built it */
    uint32_t abi_version;               /* HAKO_EXTERNAL_SENSOR_PLUGIN_ABI_VERSION */
    const char* interface_revision;     /* HAKO_EXTERNAL_SENSOR_PLUGIN_INTERFACE_REVISION */
    const char* plugin_id;              /* for logs */
    const char* world_engine;           /* the engine the plugin was built for, e.g. "mujoco" */
    int32_t world_engine_version;       /* the engine version it was built with */

    void* (*create)(const HakoExternalSensorCreateV1* args, char* error, size_t error_size);
    int32_t (*needs_main_thread)(void* sensor);
    int32_t (*register_services)(void* sensor, char* error, size_t error_size);
    int32_t (*on_start)(void* sensor, char* error, size_t error_size);
    /* sim_time_ns is the time of the physics state the sensor measures (for MuJoCo,
     * mjData.time after the physics step, in ns); it advances by physics_step_ns. */
    int32_t (*step)(void* sensor, uint64_t sim_time_ns, char* error, size_t error_size);
    int32_t (*on_stop)(void* sensor, char* error, size_t error_size);
    int32_t (*reset)(void* sensor, char* error, size_t error_size);
    int32_t (*main_thread_poll)(void* sensor, char* error, size_t error_size);
    void (*destroy)(void* sensor);
} HakoExternalSensorPluginV1;

/* The one exported symbol: returns a table in static storage. */
typedef const HakoExternalSensorPluginV1* (*HakoExternalSensorPluginEntryV1)(void);

#ifdef __cplusplus
}
#endif

#endif /* HAKONIWA_SENSOR_ADAPTER_EXTERNAL_SENSOR_PLUGIN_H */
