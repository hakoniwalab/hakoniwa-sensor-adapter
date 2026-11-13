// Entry point of the reference implementation (one translation unit per host).
#include "hakoniwa/sensor_adapter/reference/reference_implementation.hpp"

HAKO_SENSOR_ADAPTER_DEFINE_IMPLEMENTATION(reference)
{
    return &::hakoniwa::sensor_adapter::reference::implementation();
}
