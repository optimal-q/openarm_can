// Copyright 2025 Enactic, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#pragma once

#include <algorithm>
#include <chrono>
#include <cmath>
#include <mutex>

#include "gripper_component.hpp"
#include "../../canbus/can_socket.hpp"

namespace openarm::can::socket {

// High-level gripper wrapper that tracks a position target using an outer-loop PD torque
// controller with an optional torque limit.
class Gripper {
public:
    struct State {
        double position;      // measured gripper position
        double velocity;      // measured gripper velocity
        double force;         // measured torque
        bool force_holding;   // true if torque is currently being limited
        double max_effort;    // currently set maximum allowable torque (<=0 disables limiting)
    };

    explicit Gripper(canbus::CANSocket& can_socket);
    ~Gripper() = default;

    // Command position [0..1].
    void set_position(double position);
    // Set maximum allowable effort before switching to force hold.
    void set_force(double max_effort);
    // Current state snapshot.
    State get_state() const;
    // Configure position controller gains.
    void set_pid(double kp, double kd);

    // Convenience operations.
    void open();
    void close();

    inline auto get_motors() const {return component_->get_motors();}

    // Internal wiring
    void init_motor_device(damiao_motor::MotorType motor_type, uint32_t send_can_id,
                           uint32_t recv_can_id, bool use_fd);
    void update();

private:
    friend class OpenArm;

    GripperComponent& component() { return *component_; }

    std::unique_ptr<GripperComponent> component_;

    struct FirstOrderLowPass {
        double time_constant_s{0.02};

        double update(double input, std::chrono::steady_clock::time_point now) {
            if (!initialized) {
                reset(input, now);
                return state;
            }

            const std::chrono::duration<double> dt = now - last_time;
            last_time = now;

            const double dt_s = std::clamp(dt.count(), 0.0, 0.1);
            const double tau_s = time_constant_s;
            if (!(tau_s > 0.0) || !std::isfinite(tau_s)) {
                state = input;
                return state;
            }

            const double alpha = dt_s / (tau_s + dt_s);
            state += alpha * (input - state);
            return state;
        }

        void reset(double value, std::chrono::steady_clock::time_point now) {
            state = value;
            last_time = now;
            initialized = true;
        }

    private:
        bool initialized{false};
        double state{0.0};
        std::chrono::steady_clock::time_point last_time{};
    };

    mutable std::mutex state_mutex_;
    double desired_position_{0.0};
    double desired_max_effort_{2};
    double position_kp_{5.0};
    double position_kd_{0.25};
    bool force_holding_{false};
    FirstOrderLowPass motor_velocity_filter_{};
};

}  // namespace openarm::can::socket
