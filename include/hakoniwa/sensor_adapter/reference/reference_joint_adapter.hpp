#pragma once

// Reference joint sensor adapter (encoder and joint torque): returns the true
// joint values, nothing else.
//
// It follows the joint sensor specification (docs/spec/joint.md) for the time
// rules (JNT-ADP-020, -030). On each sampling step it returns the true position
// and velocity (JNT-ADP-290 and -310 without an encoder, velocity "true") and the
// true joint torque as the effort: the transmitted torque, or the actuator torque
// when the host has no transmitted torque (as JNT-ADP-410 "sensor").
//
// It adds no error of any kind: no encoder counts, delay, deflection, backlash,
// offset, cyclic error, noise, wrap, velocity from differences or LSB, and no
// torque errors, range or resolution. A definition that asks for one of them is
// refused, naming the key.

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "hakoniwa/sensor_adapter/joint_sensor_adapter.hpp"
#include "hakoniwa/sensor_adapter/reference/reference_definition.hpp"

namespace hakoniwa::sensor_adapter::reference {

class ReferenceJointAdapter final : public IJointSensorAdapter {
public:
    /// Creates the adapter from setup.config_path. Returns nullptr and sets error on failure.
    static std::unique_ptr<IJointSensorAdapter> create(const JointSensorAdapterSetup& setup, std::string& error)
    {
        auto adapter = std::unique_ptr<ReferenceJointAdapter>(new ReferenceJointAdapter());
        if (!adapter->configure(setup, error)) {
            return nullptr;
        }
        return adapter;
    }

    bool step(const JointSensorTruth& truth, JointSensorSample& out, std::string& error) override
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
            held_.position = truth.position;
            held_.velocity = truth.velocity;
            held_.effort = truth.has_transmitted_torque ? truth.transmitted_torque : truth.actuator_torque;
            held_.has_effort = true;
            held_.encoder_count = 0;
            held_.flags = 0;
        }
        out = held_;
        out.updated = sample_now;
        ++step_index_;
        return true;
    }

    JointSensorAdapterCapabilities capabilities() const override
    {
        JointSensorAdapterCapabilities c;
        c.torque_sensor = true;            // the true joint torque as the effort
        c.deterministic_with_seed = true;  // no random values at all
        c.thread_safe_across_instances = true;
        return c;
    }

    bool reset(std::string& error) override
    {
        (void)error;
        step_index_ = 0;
        held_ = JointSensorSample{};
        return true;
    }

    const std::vector<LogColumn>& log_columns() const override { return columns_; }

    void log_values(std::vector<double>& values) const override
    {
        // true_position, true_velocity, true_torque, encoder_angle, backlash_state (JNT-ADP-610).
        values = {held_.position, held_.velocity, held_.effort, held_.position, held_.position};
    }

private:
    ReferenceJointAdapter() = default;

    bool configure(const JointSensorAdapterSetup& setup, std::string& error)
    {
        sensor_name_ = setup.sensor_name;
        physics_step_ns_ = setup.physics_step_ns;
        detail::Json def;
        const std::string& path = setup.config_path;
        double update_rate = 0.0;
        if (!detail::read_definition(path, def, error) ||
            !detail::check_keys(def, "joint", {"update_rate"}, path, error) ||
            !detail::read_number(def, "update_rate", path, update_rate, error) ||
            !detail::sample_period_steps(update_rate, physics_step_ns_, sensor_name_, period_, error)) {
            return false;
        }
        columns_ = {{"true_position", "rad"},
                    {"true_velocity", "rad/s"},
                    {"true_torque", "N m"},
                    {"encoder_angle", "rad"},
                    {"backlash_state", "rad"}};
        return true;
    }

    std::string sensor_name_;
    std::uint64_t physics_step_ns_{1000000};
    std::uint64_t period_{1};
    std::uint64_t step_index_{0};
    JointSensorSample held_{};
    std::vector<LogColumn> columns_;
};

}  // namespace hakoniwa::sensor_adapter::reference
