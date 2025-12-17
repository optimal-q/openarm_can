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

#include <algorithm>
#include <cmath>

#include <openarm/can/socket/gripper.hpp>

namespace openarm::can::socket {

Gripper::Gripper(canbus::CANSocket& can_socket)
    : component_(std::make_unique<GripperComponent>(can_socket)) {}

void Gripper::init_motor_device(damiao_motor::MotorType motor_type, uint32_t send_can_id,
                                uint32_t recv_can_id, bool use_fd) {
    component_->init_motor_device(motor_type, send_can_id, recv_can_id, use_fd);
}

void Gripper::set_position(double position) {
    std::lock_guard<std::mutex> lock(state_mutex_);
    desired_position_ = std::clamp(position, 0.0, 1.0);
}

void Gripper::set_force(double max_effort) {
    std::lock_guard<std::mutex> lock(state_mutex_);
    desired_max_effort_ = std::abs(max_effort);
}

Gripper::State Gripper::get_state() const {
    Gripper::State state{};
    const auto* motor = component_->get_motor();
    if (motor) {
        state.position = std::clamp(component_->get_measured_position(), 0.0, 1.0);
        state.velocity = component_->get_measured_velocity();
        state.force = component_->get_measured_force();
    }
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        state.force_holding = force_holding_;
        state.max_effort = desired_max_effort_;
    }
    return state;
}

void Gripper::open() { set_position(1.0); }

void Gripper::close() { set_position(0.0); }

void Gripper::set_pid(double kp, double kd) {
    std::lock_guard<std::mutex> lock(state_mutex_);
    position_kp_ = kp;
    position_kd_ = kd;
}

void Gripper::update() {
    static auto last_call = std::chrono::steady_clock::now();
    auto now = std::chrono::steady_clock::now();
    auto period = std::chrono::duration<double>(now - last_call).count();
    // std::cout << "Gripper::update() calling period: " << period << " seconds" << std::endl;
    last_call = now;

    // 1) Read latest state first (unlocked).
    const auto* motor = component_->get_motor();
    if (!motor) return;

    const double motor_position = motor->get_position();
    const double motor_velocity = motor->get_velocity();

    // Low-pass filter motor velocity for the PD velocity term.
    // const double motor_velocity_for_control =
    //     motor_velocity_filter_.update(motor_velocity, std::chrono::steady_clock::now());
    const double motor_velocity_for_control = motor_velocity;

    // 2) Fetch latest desired setpoints (locked).
    double cmd_position = 0.0;
    double tau_max = 0.0;
    double kp = 0.0;
    double kd = 0.0;
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        cmd_position = desired_position_;
        tau_max = desired_max_effort_;
        kp = position_kp_;
        kd = position_kd_;
    }

    // 3) Outer-loop PD in motor coordinates; velocity target is zero.
    const double motor_position_desired =
        component_->gripper_position_to_motor_position(std::clamp(cmd_position, 0.0, 1.0));

    const double position_error = motor_position_desired - motor_position;
    const double velocity_error = -motor_velocity_for_control;
    double tau_unclamped = kp * position_error + kd * velocity_error;
    // std::cout << "tau_unclamped: " << tau_unclamped << std::endl;
    // std::cout << "motor_velocity_for_control: " << motor_velocity_for_control << std::endl;
    // std::cout << "kp: " << kp << std::endl;
    // std::cout << "kd: " << kd << std::endl;
    // std::cout << "position_error: " << position_error << std::endl;
    // std::cout << "velocity_error: " << velocity_error << std::endl;
    if (!std::isfinite(tau_unclamped)) {
        tau_unclamped = 0.0;
    }

    // 4) Apply optional torque limiting.
    const bool limit_enabled = std::isfinite(tau_max) && tau_max > 0.0;
    const double tau_command =
        limit_enabled ? std::clamp(tau_unclamped, -tau_max, tau_max) : tau_unclamped;
    const bool torque_limited = limit_enabled && (tau_command != tau_unclamped);

    // 5) Always command torque (no mode switching).
    component_->set_torque(tau_command);

    // 6) Store reporting flags.
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        force_holding_ = torque_limited;
    }
}

}  // namespace openarm::can::socket
