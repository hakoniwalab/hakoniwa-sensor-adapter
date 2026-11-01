// Contract test of the external sensor plugin interface: a host loads the fake C plugin as a
// shared library, checks its table, and drives one sensor through the lifecycle.
#include "hakoniwa/sensor_adapter/external_sensor_plugin.h"

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>

#if defined(_WIN32)
#include <windows.h>
#else
#include <dlfcn.h>
#endif

namespace {

int failures = 0;

void check(bool condition, const char* what)
{
    if (!condition) {
        std::fprintf(stderr, "FAIL: %s\n", what);
        ++failures;
    }
}

HakoExternalSensorPluginEntryV1 load(const char* path)
{
#if defined(_WIN32)
    HMODULE lib = LoadLibraryA(path);
    if (!lib) return nullptr;
    return reinterpret_cast<HakoExternalSensorPluginEntryV1>(GetProcAddress(lib, HAKO_EXTERNAL_SENSOR_PLUGIN_ENTRY));
#else
    void* lib = dlopen(path, RTLD_NOW | RTLD_LOCAL);
    if (!lib) {
        std::fprintf(stderr, "dlopen: %s\n", dlerror());
        return nullptr;
    }
    return reinterpret_cast<HakoExternalSensorPluginEntryV1>(dlsym(lib, HAKO_EXTERNAL_SENSOR_PLUGIN_ENTRY));
#endif
}

HakoExternalSensorCreateV1 args(const char* kind)
{
    HakoExternalSensorCreateV1 a{};
    a.struct_size = sizeof(a);
    a.kind = kind;
    a.config_path = "unused.json";
    a.vehicle_name = "drone";
    a.sensor_name = "front";
    a.physics_step_ns = 1000000;
    a.world_engine = "fake";
    a.world_engine_version = 1;
    a.mount_orientation = {1.0, 0.0, 0.0, 0.0};
    return a;
}

}  // namespace

int main()
{
    const auto entry = load(HAKO_FAKE_PLUGIN_PATH);
    check(entry != nullptr, "the entry symbol is exported");
    if (!entry) return EXIT_FAILURE;
    const HakoExternalSensorPluginV1* p = entry();
    check(p->struct_size >= sizeof(HakoExternalSensorPluginV1), "table size");
    check(p->abi_version == HAKO_EXTERNAL_SENSOR_PLUGIN_ABI_VERSION, "ABI version");
    check(std::strcmp(p->interface_revision, HAKO_EXTERNAL_SENSOR_PLUGIN_INTERFACE_REVISION) == 0, "interface revision");
    check(std::strcmp(p->world_engine, "fake") == 0, "world engine");

    char error[64] = "";
    // A full lifecycle with a step every 1 ms, a stop and a restart.
    auto a = args("lidar");
    void* s = p->create(&a, error, sizeof(error));
    check(s != nullptr, "create");
    check(p->needs_main_thread(s) == 0, "a LiDAR does not need the main thread");
    check(p->step(s, 0, error, sizeof(error)) != 0 && std::strlen(error) > 0, "step before start is refused with a message");
    check(p->register_services(s, error, sizeof(error)) == 0, "register_services");
    check(p->on_start(s, error, sizeof(error)) == 0, "on_start");
    bool ok = true;
    for (std::uint64_t k = 0; k < 100; ++k) ok &= p->step(s, k * 1000000, error, sizeof(error)) == 0;
    check(ok, "100 steps");
    check(p->step(s, 5, error, sizeof(error)) != 0, "a time jump is refused");
    check(p->on_stop(s, error, sizeof(error)) == 0, "on_stop");
    check(p->reset(s, error, sizeof(error)) == 0, "reset");
    check(p->on_start(s, error, sizeof(error)) == 0 && p->step(s, 0, error, sizeof(error)) == 0, "restart from time 0");
    p->destroy(s);

    // A camera relies on the main thread.
    auto c = args("rgb_camera");
    void* cam = p->create(&c, error, sizeof(error));
    p->register_services(cam, error, sizeof(error));
    p->on_start(cam, error, sizeof(error));
    check(p->needs_main_thread(cam) == 1, "a camera needs the main thread");
    check(p->step(cam, 0, error, sizeof(error)) != 0, "without main_thread_poll the camera refuses to step");
    check(p->main_thread_poll(cam, error, sizeof(error)) == 0 && p->step(cam, 0, error, sizeof(error)) == 0,
          "after main_thread_poll it steps");
    p->destroy(cam);

    // Errors: unknown kind, a smaller struct than expected, truncated messages.
    auto bad = args("sonar");
    check(p->create(&bad, error, sizeof(error)) == nullptr && std::strlen(error) > 0, "unknown kind is refused");
    auto old = args("lidar");
    old.struct_size = 8;
    check(p->create(&old, error, sizeof(error)) == nullptr, "a smaller struct is refused");
    char tiny[8];
    std::memset(tiny, 'x', sizeof(tiny));
    p->create(&bad, tiny, sizeof(tiny));
    check(std::memchr(tiny, '\0', sizeof(tiny)) != nullptr, "messages are truncated within error_size");

    if (failures == 0) std::printf("external_sensor_plugin_contract: OK\n");
    return failures == 0 ? EXIT_SUCCESS : EXIT_FAILURE;
}
