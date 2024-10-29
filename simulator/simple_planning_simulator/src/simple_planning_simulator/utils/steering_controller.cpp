// Copyright 2024 The Autoware Foundation.
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

#include "simple_planning_simulator/utils/steering_controller.hpp"

#include <algorithm>
#include <cmath>
#include <deque>
#include <iostream>
#include <map>
#include <numeric>
#include <optional>
#include <utility>
#include <vector>

namespace autoware::simple_planning_simulator::utils::steering_controller
{

using DelayBuffer = std::deque<std::pair<double, double>>;
using DelayOutput = std::pair<std::optional<double>, DelayBuffer>;

DelayOutput delay(
  const double signal, const double delay_time, const DelayBuffer & buffer,
  const double elapsed_time)
{
  DelayBuffer new_buffer = buffer;

  new_buffer.push_back(std::make_pair(signal, elapsed_time));

  if (!buffer.empty() && (elapsed_time - new_buffer.front().second) >= delay_time) {
    const double delayed_signal = new_buffer.front().first;
    new_buffer.pop_front();
    return {delayed_signal, new_buffer};
  } else {
    return {std::nullopt, new_buffer};
  }
}

double sign(const double x)
{
  return (x >= 0.0) ? 1.0 : -1.0;
}

double apply_limits(
  const double current_angle, const double previous_angle, const double angle_limit,
  const double rate_limit, const double dt)
{
  const double angle_diff = std::clamp(current_angle, -angle_limit, angle_limit) - previous_angle;
  const double rate_limited_diff = std::clamp(angle_diff, -rate_limit * dt, rate_limit * dt);
  return std::clamp(previous_angle + rate_limited_diff, -angle_limit, angle_limit);
}

double feedforward(const double input_angle, const double ff_gain)
{
  return ff_gain * input_angle;
}

double polynomial_transform(
  const double torque, const double speed, const double a, const double b, const double c,
  const double d, const double e, const double f, const double g, const double h)
{
  return a * torque * torque * torque + b * torque * torque + c * torque +
         d * speed * speed * speed + e * speed * speed + f * speed + g * torque * speed + h;
}

PIDController::PIDController(const double kp, const double ki, const double kd)
: kp_(kp), ki_(ki), kd_(kd), state_{0.0, 0.0}
{
}

double PIDController::compute(
  const double error, const double integral, const double prev_error, const double dt) const
{
  const double p_term = kp_ * error;
  const double i_term = ki_ * integral;
  const double d_term = dt < 1e-6 ? 0.0 : kd_ * (error - prev_error) / dt;

  std::cerr << "p_term: " << p_term << " i_term: " << i_term << " d_term: " << d_term << " dt: " << dt
            << std::endl;
  return p_term + i_term + d_term;
}

void PIDController::update_state(const double error, const double dt)
{
  state_.integral += error * dt;
  state_.error = error;
};

PIDControllerState PIDController::get_state() const
{
  return state_;
}

void PIDController::clear_state()
{
  state_ = {0.0, 0.0};
}

SteeringDynamics::SteeringDynamics(
  const double angular_position, const double angular_velocity, const double inertia,
  const double damping, const double stiffness, const double friction,
  const double dead_zone_threshold)
: state_{angular_position, angular_velocity, false},
  inertia_(inertia),
  damping_(damping),
  stiffness_(stiffness),
  friction_(friction),
  dead_zone_threshold_(dead_zone_threshold)
{
}

bool SteeringDynamics::is_in_dead_zone(
  const SteeringDynamicsState & state, const double input_torque) const
{
  bool is_in_dead_zone = state.is_in_dead_zone;
  const int rotation_direction = sign(state.angular_velocity);
  const int input_direction = sign(input_torque);

  if (input_direction != rotation_direction && std::abs(input_torque) < dead_zone_threshold_) {
    return true;
  }

  if (is_in_dead_zone) {
    return !(dead_zone_threshold_ <= std::abs(input_torque));
  }

  return is_in_dead_zone;
}

SteeringDynamicsDeltaState SteeringDynamics::calc_model(
  const SteeringDynamicsState & state, const double input_torque) const
{
  const double friction_force = friction_ * sign(state.angular_velocity);
  const double angular_acceleration = (input_torque - damping_ * state.angular_velocity -
                                       stiffness_ * state.angular_position - friction_force) /
                                      inertia_;

  const double d_angular_velocity = angular_acceleration;
  const double d_angular_position = state.angular_velocity;

  return {d_angular_position, d_angular_velocity};
}

void SteeringDynamics::set_state(const SteeringDynamicsState & state)
{
  state_ = state;
}

SteeringDynamicsState SteeringDynamics::get_state() const
{
  return state_;
}

void SteeringDynamics::set_steer(const double steer)
{
  state_.angular_position = steer;
}

void SteeringDynamics::clear_state()
{
  state_ = {0.0, 0.0, false};
}

SteeringController::SteeringController(
  const PIDControllerParams & pid_params, const SteeringDynamicsParams & dynamics_params,
  const std::map<std::string, double> & params)
: pid_(pid_params.kp, pid_params.ki, pid_params.kd),
  steering_dynamics_(
    dynamics_params.angular_position, dynamics_params.angular_velocity, dynamics_params.inertia,
    dynamics_params.damping, dynamics_params.stiffness, dynamics_params.friction,
    dynamics_params.dead_zone_threshold),
  params_(params)
{
}

void SteeringController::set_steer(const double steer)
{
  steering_dynamics_.set_steer(steer);
}

double SteeringController::update_euler(
  const double input_angle, const double speed, const double prev_input_angle, const double dt)
{
  const auto dynamics_state = steering_dynamics_.get_state();

  const auto d_state =
    run_one_step(input_angle, speed, prev_input_angle, dt, delay_buffer_, pid_, steering_dynamics_);

  const double d_angular_position = d_state.dynamics_d_state.d_angular_position;
  const double d_angular_velocity = d_state.dynamics_d_state.d_angular_velocity;

  auto dynamics_state_new = dynamics_state;
  dynamics_state_new.angular_position = std::clamp(
    dynamics_state.angular_position + d_angular_position * dt, -params_.at("angle_limit"),
    params_.at("angle_limit"));
  dynamics_state_new.angular_velocity = std::clamp(
    dynamics_state.angular_velocity + d_angular_velocity * dt, -params_.at("rate_limit"),
    params_.at("rate_limit"));
  dynamics_state_new.is_in_dead_zone = d_state.is_in_dead_zone;
  steering_dynamics_.set_state(dynamics_state_new);

  pid_.update_state(d_state.pid_error, dt);
  delay_buffer_ = d_state.delay_buffer;

  return dynamics_state_new.angular_position;
}

double SteeringController::update_runge_kutta(
  const double input_angle, const double speed, const double prev_input_angle, const double dt)
{
  const auto dynamics_state = steering_dynamics_.get_state();

  const auto k1 =
    run_one_step(input_angle, speed, prev_input_angle, dt, delay_buffer_, pid_, steering_dynamics_);

  auto dynamics_for_k2 = steering_dynamics_;
  auto dynamics_state_for_k2 = steering_dynamics_.get_state();
  dynamics_state_for_k2.angular_position =
    dynamics_state.angular_position + k1.dynamics_d_state.d_angular_position * dt / 2.0;
  dynamics_state_for_k2.angular_position =
    dynamics_state.angular_position + k1.dynamics_d_state.d_angular_position * dt / 2.0;
  dynamics_state_for_k2.angular_velocity =
    dynamics_state.angular_velocity + k1.dynamics_d_state.d_angular_velocity * dt / 2.0;
  dynamics_for_k2.set_state(dynamics_state_for_k2);
  const auto k2 =
    run_one_step(input_angle, speed, prev_input_angle, dt, delay_buffer_, pid_, dynamics_for_k2);

  auto dynamics_for_k3 = steering_dynamics_;
  auto dynamics_state_for_k3 = steering_dynamics_.get_state();
  dynamics_state_for_k3.angular_position =
    dynamics_state.angular_position + k2.dynamics_d_state.d_angular_position * dt / 2.0;
  dynamics_state_for_k3.angular_velocity =
    dynamics_state.angular_velocity + k2.dynamics_d_state.d_angular_velocity * dt / 2.0;
  dynamics_for_k3.set_state(dynamics_state_for_k3);
  const auto k3 =
    run_one_step(input_angle, speed, prev_input_angle, dt, delay_buffer_, pid_, dynamics_for_k3);

  auto dynamics_for_k4 = steering_dynamics_;
  auto dynamics_state_for_k4 = steering_dynamics_.get_state();
  dynamics_state_for_k4.angular_position =
    dynamics_state.angular_position + k3.dynamics_d_state.d_angular_position * dt;
  dynamics_state_for_k4.angular_velocity =
    dynamics_state.angular_velocity + k3.dynamics_d_state.d_angular_velocity * dt;
  dynamics_for_k4.set_state(dynamics_state_for_k4);
  const auto k4 =
    run_one_step(input_angle, speed, prev_input_angle, dt, delay_buffer_, pid_, dynamics_for_k4);

  const double d_angular_position =
    (k1.dynamics_d_state.d_angular_position + 2.0 * k2.dynamics_d_state.d_angular_position +
     2.0 * k3.dynamics_d_state.d_angular_position + k4.dynamics_d_state.d_angular_position) /
    6.0;
  const double d_angular_velocity =
    (k1.dynamics_d_state.d_angular_velocity + 2.0 * k2.dynamics_d_state.d_angular_velocity +
     2.0 * k3.dynamics_d_state.d_angular_velocity + k4.dynamics_d_state.d_angular_velocity) /
    6.0;

  auto dynamics_state_new = dynamics_state;
  dynamics_state_new.angular_position = std::clamp(
    dynamics_state.angular_position + d_angular_position * dt, -params_.at("angle_limit"),
    params_.at("angle_limit"));
  dynamics_state_new.angular_velocity = std::clamp(
    dynamics_state.angular_velocity + d_angular_velocity * dt, -params_.at("rate_limit"),
    params_.at("rate_limit"));
  dynamics_state_new.is_in_dead_zone = k4.is_in_dead_zone;
  steering_dynamics_.set_state(dynamics_state_new);

  pid_.update_state(k4.pid_error, dt);
  delay_buffer_ = k4.delay_buffer;

  return dynamics_state_new.angular_position;
}

StepResult SteeringController::run_one_step(
  const double input_angle, const double speed, const double prev_input_angle, const double dt,
  const DelayBuffer & delay_buffer, const PIDController & pid,
  const SteeringDynamics & dynamics) const
{
  const auto dynamics_state = dynamics.get_state();
  const auto pid_state = pid.get_state();

  const double limited_input_angle = apply_limits(
    input_angle, prev_input_angle, params_.at("angle_limit"), params_.at("rate_limit"), dt);

  const double ff_torque = feedforward(limited_input_angle, params_.at("ff_gain"));

  const double pid_error = limited_input_angle - dynamics_state.angular_position;

  const double pid_torque =
    pid.compute(pid_error, pid_state.integral + pid_error * dt, pid_state.error, dt);

  const double total_torque = ff_torque + pid_torque;

  const double steering_torque = std::clamp(
    polynomial_transform(
      total_torque, speed, params_.at("a"), params_.at("b"), params_.at("c"), params_.at("d"),
      params_.at("e"), params_.at("f"), params_.at("g"), params_.at("h")),
    -30.0, 30.0);

  const double elapsed_time = delay_buffer.empty() ? dt : delay_buffer.back().second + dt;
  const auto delay_output =
    delay(steering_torque, params_.at("delay_time"), delay_buffer, elapsed_time);

  const auto delay_buffer_new = delay_output.second;
  const auto delayed_torque_opt = delay_output.first;
  if (!delayed_torque_opt.has_value()) {
    return {delay_buffer_new, pid_error, {0.0, 0.0}, dynamics_state.is_in_dead_zone};
  }

  const bool is_in_dead_zone = dynamics.is_in_dead_zone(dynamics_state, delayed_torque_opt.value());
  if (is_in_dead_zone) {
    return {delay_buffer_new, pid_error, {0.0, 0.0}, true};
  }

  const auto d_state = dynamics.calc_model(dynamics.get_state(), delayed_torque_opt.value());
  // const auto d_state = dynamics.calc_model(dynamics.get_state(), steering_torque);

  // debug print
  std::cerr << "input_angle: " << input_angle << std::endl;
  std::cerr << "limited_input_angle: " << limited_input_angle << std::endl;
  std::cerr << "ff_torque: " << ff_torque << std::endl;
  std::cerr << "pid_error: " << pid_error << std::endl;
  std::cerr << "pid_torque: " << pid_torque << std::endl;
  std::cerr << "total_torque: " << total_torque << std::endl;
  std::cerr << "steering_torque: " << steering_torque << std::endl;
  // std::cerr << "delayed_torque: " << delayed_torque_opt.value() << std::endl;
  std::cerr << "d_angular_position: " << d_state.d_angular_position << std::endl;
  std::cerr << "d_angular_velocity: " << d_state.d_angular_velocity << std::endl;


  return {delay_buffer_new, pid_error, d_state, false};
}

}  // namespace autoware::simple_planning_simulator::utils::steering_controller
