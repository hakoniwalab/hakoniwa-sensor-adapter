#pragma once

// Reference force-torque adapter: returns the true wrench, nothing else.
//
// It follows the force-torque specification (docs/spec/force-torque.md) for the
// time rules (FTS-ADP-020, -030) and returns the truth on each sampling step
// (FTS-ADP-300, no stage written).
//
// It adds no error of any kind: no measurement range (saturation), overload,
// delay, low-pass, nonlinearity, calibration errors, bias, drift, noise,
// resolution, and no tare. A definition that asks for one of them is refused,
// naming the key.

#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "hakoniwa/sensor_adapter/force_torque_sensor_adapter.hpp"
#include "hakoniwa/sensor_adapter/reference/reference_definition.hpp"

namespace hakoniwa::sensor_adapter::reference {

class ReferenceForceTorqueAdapter final : public IForceTorqueSensorAdapter {
public:
    /// Creates the adapter from setup.config_path. Returns nullptr and sets error on failure.
    static std::unique_ptr<IForceTorqueSensorAdapter> create(const ForceTorqueAdapterSetup& setup,
                                                             std::string& error)
    {
        auto adapter = std::unique_ptr<ReferenceForceTorqueAdapter>(new ReferenceForceTorqueAdapter());
        if (!adapter->configure(setup, error)) {
            return nullptr;
        }
        return adapter;
    }

    bool step(const ForceTorqueTruth& truth, ForceTorqueSample& out, std::string& error) override
    {
        const std::uint64_t expected = step_index_ * physics_step_ns_;
        if (truth.sim_time_ns != expected) {
            error = sensor_name_ + ": step time " + std::to_string(truth.sim_time_ns) + " ns, expected " +
                    std::to_string(expected) + " ns";
            return false;
        }
        const bool sample_now = (step_index_ % period_) == 0;
        if (sample_now) {
            held_.sample_time_ns = truth.sim_time_ns;
            held_.wrench = truth.wrench;
            held_.flags = 0;
        }
        out = held_;
        out.updated = sample_now;
        ++step_index_;
        return true;
    }

    bool tare(std::string& error) override
    {
        error = sensor_name_ + ": the reference implementation returns the true wrench and has no tare";
        return false;
    }

    void clear_tare() override {}

    ForceTorqueAdapterCapabilities capabilities() const override
    {
        ForceTorqueAdapterCapabilities c;
        c.deterministic_with_seed = true;  // no random values at all
        c.thread_safe_across_instances = true;
        return c;
    }

    bool reset(std::string& error) override
    {
        (void)error;
        step_index_ = 0;
        held_ = ForceTorqueSample{};
        return true;
    }

    const std::vector<LogColumn>& log_columns() const override { return columns_; }

    void log_values(std::vector<double>& values) const override
    {
        // true_fx..true_tz, drift_fx..drift_tz and tare_fx..tare_tz (FTS-ADP-610; drift and tare are 0).
        const Wrench& w = held_.wrench;
        values = {w.force.x, w.force.y, w.force.z, w.torque.x, w.torque.y, w.torque.z};
        values.insert(values.end(), 12, 0.0);
    }

private:
    ReferenceForceTorqueAdapter() = default;

    bool configure(const ForceTorqueAdapterSetup& setup, std::string& error)
    {
        sensor_name_ = setup.sensor_name;
        physics_step_ns_ = setup.physics_step_ns;
        detail::Json def;
        const std::string& path = setup.config_path;
        double update_rate = 0.0;
        if (!detail::read_definition(path, def, error) ||
            !detail::check_keys(def, "force_torque", {"update_rate"}, path, error) ||
            !detail::read_number(def, "update_rate", path, update_rate, error) ||
            !detail::sample_period_steps(update_rate, physics_step_ns_, sensor_name_, period_, error)) {
            return false;
        }
        columns_.clear();
        for (const char* group : {"true", "drift", "tare"}) {
            for (const char* axis : {"fx", "fy", "fz", "tx", "ty", "tz"}) {
                columns_.push_back({std::string(group) + "_" + axis, axis[0] == 'f' ? "N" : "N m"});
            }
        }
        return true;
    }

    std::string sensor_name_;
    std::uint64_t physics_step_ns_{1000000};
    std::uint64_t period_{1};
    std::uint64_t step_index_{0};
    ForceTorqueSample held_{};
    std::vector<LogColumn> columns_;
};

}  // namespace hakoniwa::sensor_adapter::reference
