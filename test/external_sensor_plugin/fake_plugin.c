/* A fake external sensor plugin, written in C to prove the interface needs nothing else.
 * It checks that the host follows the lifecycle order and reports violations. */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "hakoniwa/sensor_adapter/external_sensor_plugin.h"

enum Stage { CREATED, SERVICES, STARTED, STOPPED };

typedef struct FakeSensor {
    enum Stage stage;
    uint64_t next_ns;
    uint64_t step_ns;
    int camera;
    int main_polls;
} FakeSensor;

static void set_error(char* error, size_t size, const char* text)
{
    if (error && size > 0) snprintf(error, size, "%s", text);
}

static void* fake_create(const HakoExternalSensorCreateV1* a, char* error, size_t size)
{
    if (!a || a->struct_size < sizeof(HakoExternalSensorCreateV1)) {
        set_error(error, size, "create: struct_size is smaller than this plugin expects");
        return NULL;
    }
    if (!a->kind || (strcmp(a->kind, "lidar") != 0 && strcmp(a->kind, "rgb_camera") != 0)) {
        set_error(error, size, "create: unsupported kind");
        return NULL;
    }
    if (!a->world_engine || strcmp(a->world_engine, "fake") != 0 || a->physics_step_ns == 0) {
        set_error(error, size, "create: this plugin needs the fake world and a physics step");
        return NULL;
    }
    FakeSensor* s = (FakeSensor*)calloc(1, sizeof(FakeSensor));
    s->stage = CREATED;
    s->step_ns = a->physics_step_ns;
    s->camera = strcmp(a->kind, "rgb_camera") == 0;
    return s;
}

static int32_t fake_needs_main_thread(void* p) { return ((FakeSensor*)p)->camera; }

static int32_t expect(FakeSensor* s, enum Stage want, enum Stage next, const char* what, char* error, size_t size)
{
    if (s->stage != want) {
        set_error(error, size, what);
        return 1;
    }
    s->stage = next;
    return 0;
}

static int32_t fake_register_services(void* p, char* e, size_t n)
{
    return expect((FakeSensor*)p, CREATED, SERVICES, "register_services must come right after create", e, n);
}

static int32_t fake_on_start(void* p, char* e, size_t n)
{
    FakeSensor* s = (FakeSensor*)p;
    if (s->stage == STOPPED) s->stage = SERVICES;  /* a restart after a stop */
    return expect(s, SERVICES, STARTED, "on_start must come after register_services", e, n);
}

static int32_t fake_step(void* p, uint64_t t, char* e, size_t n)
{
    FakeSensor* s = (FakeSensor*)p;
    if (s->stage != STARTED) {
        set_error(e, n, "step is allowed only while running");
        return 1;
    }
    if (t != s->next_ns) {
        set_error(e, n, "sim_time_ns must advance by physics_step_ns from 0");
        return 1;
    }
    if (s->camera && s->main_polls == 0) {
        set_error(e, n, "a camera needs main_thread_poll before its first step");
        return 1;
    }
    s->next_ns += s->step_ns;
    return 0;
}

static int32_t fake_on_stop(void* p, char* e, size_t n)
{
    return expect((FakeSensor*)p, STARTED, STOPPED, "on_stop must come while running", e, n);
}

static int32_t fake_reset(void* p, char* e, size_t n)
{
    (void)e;
    (void)n;
    ((FakeSensor*)p)->next_ns = 0;
    return 0;
}

static int32_t fake_main_thread_poll(void* p, char* e, size_t n)
{
    (void)e;
    (void)n;
    ((FakeSensor*)p)->main_polls++;
    return 0;
}

static void fake_destroy(void* p) { free(p); }

static const HakoExternalSensorPluginV1 kTable = {
    sizeof(HakoExternalSensorPluginV1),
    HAKO_EXTERNAL_SENSOR_PLUGIN_ABI_VERSION,
    HAKO_EXTERNAL_SENSOR_PLUGIN_INTERFACE_REVISION,
    "fake-plugin",
    "fake",
    1,
    fake_create,
    fake_needs_main_thread,
    fake_register_services,
    fake_on_start,
    fake_step,
    fake_on_stop,
    fake_reset,
    fake_main_thread_poll,
    fake_destroy,
};

HAKO_EXTERNAL_SENSOR_PLUGIN_EXPORT const HakoExternalSensorPluginV1* hako_external_sensor_plugin_v1(void)
{
    return &kTable;
}
