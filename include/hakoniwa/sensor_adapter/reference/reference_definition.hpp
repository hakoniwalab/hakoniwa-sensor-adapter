#pragma once

// Shared helpers of the reference implementation: reading a common sensor
// definition file (JSON) and the sample period.
//
// The reference implementation needs nlohmann/json (header-only) on the include
// path. The interface headers of this library do not.

#include <cmath>
#include <cstdint>
#include <cstdio>
#include <fstream>
#include <set>
#include <string>

#include <nlohmann/json.hpp>

namespace hakoniwa::sensor_adapter::reference::detail {

using Json = nlohmann::json;

/// Reads a definition file. Returns false and sets error on failure.
inline bool read_definition(const std::string& path, Json& out, std::string& error)
{
    std::ifstream in(path);
    if (!in) {
        error = "cannot open the sensor definition: " + path;
        return false;
    }
    try {
        in >> out;
    } catch (const std::exception& e) {
        error = "cannot parse the sensor definition " + path + ": " + e.what();
        return false;
    }
    if (!out.is_object()) {
        error = "the sensor definition is not a JSON object: " + path;
        return false;
    }
    return true;
}

/// Checks "type" and that every key is either supported or "$schema". A key the
/// reference implementation does not provide (for example noise) is an error
/// naming the key: the definition asks for something this implementation would
/// otherwise silently ignore.
inline bool check_keys(const Json& def, const std::string& type, const std::set<std::string>& supported,
                       const std::string& path, std::string& error)
{
    if (!def.contains("type") || !def.at("type").is_string() || def.at("type").get<std::string>() != type) {
        error = "the sensor definition " + path + " is not of type \"" + type + "\"";
        return false;
    }
    for (const auto& item : def.items()) {
        const std::string& key = item.key();
        if (key == "type" || key == "$schema" || supported.count(key) != 0) {
            continue;
        }
        error = "the reference implementation does not provide \"" + key + "\" (" + path +
                "); remove it or choose another implementation";
        return false;
    }
    return true;
}

/// Reads a finite number. Returns false and sets error when missing or not a number.
inline bool read_number(const Json& object, const char* key, const std::string& where, double& out,
                        std::string& error)
{
    if (!object.contains(key) || !object.at(key).is_number()) {
        error = where + ": \"" + key + "\" must be a number";
        return false;
    }
    out = object.at(key).get<double>();
    if (!std::isfinite(out)) {
        error = where + ": \"" + key + "\" must be finite";
        return false;
    }
    return true;
}

/// The sample period in physics steps: the period of update_rate [Hz] rounded to a
/// whole number of steps (round half up, at least 1). Warns on standard error when
/// the period is not an exact multiple of the physics step (RIS-COM-060, FTS-ADP-030).
inline bool sample_period_steps(double update_rate, std::uint64_t physics_step_ns, const std::string& sensor,
                                std::uint64_t& out, std::string& error)
{
    if (!(update_rate > 0.0)) {
        error = sensor + ": update_rate must be greater than 0";
        return false;
    }
    if (physics_step_ns == 0) {
        error = sensor + ": physics_step_ns must be greater than 0";
        return false;
    }
    const double steps = (1.0e9 / update_rate) / static_cast<double>(physics_step_ns);
    const double rounded = std::floor(steps + 0.5);
    out = rounded < 1.0 ? 1 : static_cast<std::uint64_t>(rounded);
    if (std::fabs(steps - static_cast<double>(out)) > 1e-9) {
        std::fprintf(stderr,
                     "[WARN] %s: the sample period (%.9g steps) is not a whole number of physics steps; "
                     "using %llu steps\n",
                     sensor.c_str(), steps, static_cast<unsigned long long>(out));
    }
    return true;
}

}  // namespace hakoniwa::sensor_adapter::reference::detail
