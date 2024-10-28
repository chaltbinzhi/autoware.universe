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

#ifndef SIMPLE_PLANNING_SIMULATOR__UTILS__STEERING_CONTROLLER_HPP_
#define SIMPLE_PLANNING_SIMULATOR__UTILS__STEERING_CONTROLLER_HPP_

#include <deque>
#include <map>
#include <optional>
#include <string>
#include <utility>

namespace autoware::simple_planning_simulator::utils::steering_controller
{

using DelayBuffer = std::deque<std::pair<double, double>>;
using DelayOutput = std::pair<std::optional<double>, DelayBuffer>;

DelayOutput delay(
  const double signal, const double delay_time, const DelayBuffer & buffer,
  const double elapsed_time);

double sign(const double x);

double apply_limits(
  const double current_angle, const double previous_angle, const double angle_limit,
  const double rate_limit, const double dt);

double feedforward(const double input_angle, const double ff_gain);

double polynomial_transform(
  const double torque, const double speed, const double a, const double b, const double c,
  const double d, const double e, const double f, const double g, const double h);

struct PIDControllerParams
{
  double kp{0.0};
  double ki{0.0};
  double kd{0.0};
};

struct PIDControllerState
{
  double integral;
  double error;
};

class PIDController
{
public:
  PIDController() = default;
  PIDController(const double kp, const double ki, const double kd);

  [[nodiscard]] double compute(
    const double error, const double integral, const double prev_error, const double dt) const;

  void update_state(const double error, const double dt);

  [[nodiscard]] PIDControllerState get_state() const;

  void clear_state();

private:
  double kp_, ki_, kd_;
  PIDControllerState state_;
};

struct SteeringDynamicsParams
{
  double angular_position{0.0};
  double angular_velocity{0.0};
  double inertia{0.0};
  double damping{0.0};
  double stiffness{0.0};
  double friction{0.0};
  double dead_zone_threshold{0.0};
};

struct SteeringDynamicsState
{
  double angular_position;
  double angular_velocity;
  bool is_in_dead_zone;
};

struct SteeringDynamicsDeltaState
{
  double d_angular_position;
  double d_angular_velocity;
};

class SteeringDynamics
{
public:
  SteeringDynamics() = default;
  SteeringDynamics(
    const double angular_position, const double angular_velocity, const double inertia,
    const double damping, const double stiffness, const double friction,
    const double dead_zone_threshold);

  [[nodiscard]] bool is_in_dead_zone(
    const SteeringDynamicsState & state, const double input_torque) const;

  [[nodiscard]] SteeringDynamicsDeltaState calc_model(
    const SteeringDynamicsState & state, const double input_torque) const;

  void set_state(const SteeringDynamicsState & state);

  [[nodiscard]] SteeringDynamicsState get_state() const;

  void clear_state();

private:
  SteeringDynamicsState state_;
  double inertia_;
  double damping_;
  double stiffness_;
  double friction_;
  double dead_zone_threshold_;
};

struct StepResult
{
  DelayBuffer delay_buffer;
  double pid_error;
  SteeringDynamicsDeltaState dynamics_d_state;
  bool is_in_dead_zone;
};

class SteeringController
{
public:
  SteeringController() = default;
  SteeringController(
    const PIDControllerParams & pid_params, const SteeringDynamicsParams & dynamics_params,
    const std::map<std::string, double> & params);

  double update_kutta_update(
    const double input_angle, const double speed, const double prev_input_angle, const double dt);

private:
  DelayBuffer delay_buffer_;
  PIDController pid_;
  SteeringDynamics steering_dynamics_;
  std::map<std::string, double> params_;

  [[nodiscard]] StepResult run_one_step(
    const double input_angle, const double speed, const double prev_input_angle, const double dt,
    const DelayBuffer & delay_buffer, const PIDController & pid,
    const SteeringDynamics & dynamics) const;
};

}  // namespace autoware::simple_planning_simulator::utils::steering_controller

#endif  // SIMPLE_PLANNING_SIMULATOR__UTILS__STEERING_CONTROLLER_HPP_
