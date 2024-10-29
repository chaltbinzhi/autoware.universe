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

#include "simple_planning_simulator/vehicle_model/sim_model_actuation_cmd.hpp"

#include "autoware_vehicle_msgs/msg/gear_command.hpp"

#include <algorithm>

bool ActuationMap::readActuationMapFromCSV(const std::string & csv_path, const bool validation)
{
  CSVLoader csv(csv_path);
  std::vector<std::vector<std::string>> table;

  if (!csv.readCSV(table)) {
    return false;
  }

  state_index_ = CSVLoader::getColumnIndex(table);
  actuation_index_ = CSVLoader::getRowIndex(table);
  actuation_map_ = CSVLoader::getMap(table);
  return !validation || CSVLoader::validateMap(actuation_map_, true);
}

double ActuationMap::getControlCommand(const double actuation, const double state) const
{
  std::vector<double> interpolated_control_vec{};
  const double clamped_state = CSVLoader::clampValue(state, state_index_);

  interpolated_control_vec.reserve(actuation_map_.size());
  for (const std::vector<double> & control_command_values : actuation_map_) {
    interpolated_control_vec.push_back(
      autoware::interpolation::lerp(state_index_, control_command_values, clamped_state));
  }

  const double clamped_actuation = CSVLoader::clampValue(actuation, actuation_index_);
  return autoware::interpolation::lerp(
    actuation_index_, interpolated_control_vec, clamped_actuation);
}

std::optional<double> AccelMap::getThrottle(const double acc, double vel) const
{
  const std::vector<double> & vel_indices = state_index_;
  const std::vector<double> & throttle_indices = actuation_index_;
  const std::vector<std::vector<double>> & accel_map = actuation_map_;

  std::vector<double> interpolated_acc_vec;
  const double clamped_vel = CSVLoader::clampValue(vel, vel_indices);

  // (throttle, vel, acc) map => (throttle, acc) map by fixing vel
  interpolated_acc_vec.reserve(accel_map.size());
  for (const std::vector<double> & accelerations : accel_map) {
    interpolated_acc_vec.push_back(
      autoware::interpolation::lerp(vel_indices, accelerations, clamped_vel));
  }

  // calculate throttle
  // When the desired acceleration is smaller than the throttle area, return null => brake sequence
  // When the desired acceleration is greater than the throttle area, return max throttle
  if (acc < interpolated_acc_vec.front()) {
    return std::nullopt;
  }
  if (interpolated_acc_vec.back() < acc) {
    return throttle_indices.back();
  }

  return autoware::interpolation::lerp(interpolated_acc_vec, throttle_indices, acc);
}

double BrakeMap::getBrake(const double acc, double vel) const
{
  const std::vector<double> & vel_indices = state_index_;
  const std::vector<double> & brake_indices = actuation_index_;
  const std::vector<std::vector<double>> & brake_map = actuation_map_;

  std::vector<double> interpolated_acc_vec;
  const double clamped_vel = CSVLoader::clampValue(vel, vel_indices);

  // (brake, vel, acc) map => (brake, acc) map by fixing vel
  interpolated_acc_vec.reserve(brake_map.size());
  for (const std::vector<double> & accelerations : brake_map) {
    interpolated_acc_vec.push_back(
      autoware::interpolation::lerp(vel_indices, accelerations, clamped_vel));
  }

  // calculate brake
  // When the desired acceleration is smaller than the brake area, return max brake on the map
  // When the desired acceleration is greater than the brake area, return min brake on the map
  if (acc < interpolated_acc_vec.back()) {
    return brake_indices.back();
  }
  if (interpolated_acc_vec.front() < acc) {
    return brake_indices.front();
  }

  std::reverse(std::begin(interpolated_acc_vec), std::end(interpolated_acc_vec));
  return autoware::interpolation::lerp(interpolated_acc_vec, brake_indices, acc);
}

// steer map sim model
SimModelActuationCmd::SimModelActuationCmd(
  double vx_lim, double steer_lim, double vx_rate_lim, double steer_rate_lim, double wheelbase,
  double dt, double accel_delay, double accel_time_constant, double brake_delay,
  double brake_time_constant, double steer_delay, double steer_time_constant, double steer_bias,
  bool convert_accel_cmd, bool convert_brake_cmd, bool convert_steer_cmd,
  std::string accel_map_path, std::string brake_map_path, std::string steer_map_path)
: SimModelInterface(6 /* dim x */, 5 /* dim u */),
  MIN_TIME_CONSTANT(0.03),
  vx_lim_(vx_lim),
  vx_rate_lim_(vx_rate_lim),
  steer_lim_(steer_lim),
  steer_rate_lim_(steer_rate_lim),
  wheelbase_(wheelbase),
  accel_delay_(accel_delay),
  accel_time_constant_(std::max(accel_time_constant, MIN_TIME_CONSTANT)),
  brake_delay_(brake_delay),
  brake_time_constant_(std::max(brake_time_constant, MIN_TIME_CONSTANT)),
  steer_delay_(steer_delay),
  steer_time_constant_(std::max(steer_time_constant, MIN_TIME_CONSTANT)),
  steer_bias_(steer_bias)
{
  initializeInputQueue(dt);
  convert_accel_cmd_ = convert_accel_cmd && accel_map_.readActuationMapFromCSV(accel_map_path);
  convert_brake_cmd_ = convert_brake_cmd && brake_map_.readActuationMapFromCSV(brake_map_path);
  convert_steer_cmd_ = convert_steer_cmd && steer_map_.readActuationMapFromCSV(steer_map_path);
  actuation_sim_type_ = ActuationSimType::STEER_MAP;
}

// VGR sim model
SimModelActuationCmd::SimModelActuationCmd(
  double vx_lim, double steer_lim, double vx_rate_lim, double steer_rate_lim, double wheelbase,
  double dt, double accel_delay, double accel_time_constant, double brake_delay,
  double brake_time_constant, double steer_delay, double steer_time_constant, double steer_bias,
  bool convert_accel_cmd, bool convert_brake_cmd, bool convert_steer_cmd,
  std::string accel_map_path, std::string brake_map_path, double vgr_coef_a, double vgr_coef_b,
  double vgr_coef_c)
: SimModelInterface(6 /* dim x */, 5 /* dim u */),
  MIN_TIME_CONSTANT(0.03),
  vx_lim_(vx_lim),
  vx_rate_lim_(vx_rate_lim),
  steer_lim_(steer_lim),
  steer_rate_lim_(steer_rate_lim),
  wheelbase_(wheelbase),
  accel_delay_(accel_delay),
  accel_time_constant_(std::max(accel_time_constant, MIN_TIME_CONSTANT)),
  brake_delay_(brake_delay),
  brake_time_constant_(std::max(brake_time_constant, MIN_TIME_CONSTANT)),
  steer_delay_(steer_delay),
  steer_time_constant_(std::max(steer_time_constant, MIN_TIME_CONSTANT)),
  steer_bias_(steer_bias),
  convert_steer_cmd_(convert_steer_cmd),
  vgr_coef_a_(vgr_coef_a),
  vgr_coef_b_(vgr_coef_b),
  vgr_coef_c_(vgr_coef_c)
{
  initializeInputQueue(dt);
  convert_accel_cmd_ = convert_accel_cmd && accel_map_.readActuationMapFromCSV(accel_map_path);
  convert_brake_cmd_ = convert_brake_cmd && brake_map_.readActuationMapFromCSV(brake_map_path);
  actuation_sim_type_ = ActuationSimType::VGR;

  const std::map<std::string, double> params = {
    {"kp", 386.9151820510161},
    {"ki", 5.460970982628869},
    {"kd", 0.03550834077602694},
    {"ff_gain", 0.03051963576179274},
    {"angle_limit", 10.0},
    {"rate_limit", 3.0},
    {"dead_zone_threshold", 0.00708241866710033},
    {"a", 0.15251276182076065},
    {"b", -0.17301900674117585},
    {"c", 1.5896528355739639},
    {"d", 0.002300899817071436},
    {"e", -0.0418928856764797},
    {"f", 0.18449047960081838},
    {"g", -0.06320887302605509},
    {"h", 0.18696796150634806},
    {"inertia", 25.17844747941984},
    {"damping", 117.00653795106054},
    {"stiffness", 0.17526182368541224},
    {"friction", 0.6596571248682918},
    {"vgr_coef_a", 2.4181735349544224},
    {"vgr_coef_b", -0.013434076966833082},
    {"vgr_coef_c", -0.033963661615283594},
    {"delay_time", 0.0007641271506616108}};

  steering_controller_ = SteeringController(
    {params.at("kp"), params.at("ki"), params.at("kd")},
    {params.at("angle_limit"), params.at("rate_limit"), params.at("inertia"), params.at("damping"),
     params.at("stiffness"), params.at("friction"), params.at("dead_zone_threshold")},
    params);
}

double SimModelActuationCmd::getX()
{
  return state_(IDX::X);
}
double SimModelActuationCmd::getY()
{
  return state_(IDX::Y);
}
double SimModelActuationCmd::getYaw()
{
  return state_(IDX::YAW);
}
double SimModelActuationCmd::getVx()
{
  return state_(IDX::VX);
}
double SimModelActuationCmd::getVy()
{
  return 0.0;
}
double SimModelActuationCmd::getAx()
{
  return state_(IDX::ACCX);
}
double SimModelActuationCmd::getWz()
{
  return state_(IDX::VX) * std::tan(state_(IDX::STEER)) / wheelbase_;
}
double SimModelActuationCmd::getSteer()
{
  return state_(IDX::STEER) + steer_bias_;
}
void SimModelActuationCmd::update(const double & dt)
{
  Eigen::VectorXd delayed_input = Eigen::VectorXd::Zero(dim_u_);

  accel_input_queue_.push_back(input_(IDX_U::ACCEL_DES));
  delayed_input(IDX_U::ACCEL_DES) = accel_input_queue_.front();
  accel_input_queue_.pop_front();

  brake_input_queue_.push_back(input_(IDX_U::BRAKE_DES));
  delayed_input(IDX_U::BRAKE_DES) = brake_input_queue_.front();
  brake_input_queue_.pop_front();

  steer_input_queue_.push_back(input_(IDX_U::STEER_DES));
  delayed_input(IDX_U::STEER_DES) = steer_input_queue_.front();
  steer_input_queue_.pop_front();

  delayed_input(IDX_U::GEAR) = input_(IDX_U::GEAR);
  delayed_input(IDX_U::SLOPE_ACCX) = input_(IDX_U::SLOPE_ACCX);

  const auto prev_state = state_;

  updateRungeKuttaWithController(dt, delayed_input);
  // updateRungeKutta(dt, delayed_input);

  // take velocity/steer limit explicitly
  state_(IDX::VX) = std::clamp(state_(IDX::VX), -vx_lim_, vx_lim_);
  state_(IDX::STEER) = std::clamp(state_(IDX::STEER), -steer_lim_, steer_lim_);

  // consider gear
  // update position and velocity first, and then acceleration is calculated naturally
  updateStateWithGear(state_, prev_state, gear_, dt);
}

void SimModelActuationCmd::initializeInputQueue(const double & dt)
{
  const size_t accel_input_queue_size = static_cast<size_t>(round(accel_delay_ / dt));
  accel_input_queue_.resize(accel_input_queue_size);
  std::fill(accel_input_queue_.begin(), accel_input_queue_.end(), 0.0);

  const size_t brake_input_queue_size = static_cast<size_t>(round(brake_delay_ / dt));
  brake_input_queue_.resize(brake_input_queue_size);
  std::fill(brake_input_queue_.begin(), brake_input_queue_.end(), 0.0);

  const size_t steer_input_queue_size = static_cast<size_t>(round(steer_delay_ / dt));
  steer_input_queue_.resize(steer_input_queue_size);
  std::fill(steer_input_queue_.begin(), steer_input_queue_.end(), 0.0);
}

void SimModelActuationCmd::updateRungeKuttaWithController(
  const double dt, const Eigen::VectorXd & input)
{
  // 1) update longitudinal states
  const Eigen::VectorXd k1_longitudinal = calcLongitudinalModel(state_, input);
  const Eigen::VectorXd k2_longitudinal =
    calcLongitudinalModel(state_ + dt / 2.0 * k1_longitudinal, input);
  const Eigen::VectorXd k3_longitudinal =
    calcLongitudinalModel(state_ + dt / 2.0 * k2_longitudinal, input);
  const Eigen::VectorXd k4_longitudinal =
    calcLongitudinalModel(state_ + dt * k3_longitudinal, input);
  state_ +=
    dt / 6.0 * (k1_longitudinal + 2.0 * k2_longitudinal + 2.0 * k3_longitudinal + k4_longitudinal);

  // 2) update lateral states
  const double steer = state_(IDX::STEER);
  const double steer_des = input(IDX_U::STEER_DES);
  const double vel = state_(IDX::VX);
  // const double k1_lateral = calcLateralModel(steer, steer_des, vel);
  // const double k2_lateral = calcLateralModel(
  //   steer + dt / 2.0 * k1_lateral, steer_des, vel + dt / 2.0 * k1_longitudinal(IDX::VX));
  // const double k3_lateral = calcLateralModel(
  //   steer + dt / 2.0 * k2_lateral, steer_des, vel + dt / 2.0 * k2_longitudinal(IDX::VX));
  // const double k4_lateral =
  //   calcLateralModel(steer + dt * k3_lateral, steer_des, vel + dt * k3_longitudinal(IDX::VX));

  // state_(IDX::STEER) += dt / 6.0 * (k1_lateral + 2.0 * k2_lateral + 2.0 * k3_lateral +
  // k4_lateral);

  const double steer_tire_des = calculateSteeringTireCommand(vel, steer, steer_des);
  std::cerr << "dt: " << dt << std::endl;
  std::cerr << "vel: " << vel << std::endl;
  std::cerr << "steer: " << steer << std::endl;
  std::cerr << "steer_des: " << steer_des << std::endl;
  std::cerr << "steer_tire_des: " << steer_tire_des << std::endl;
  const double prev_steer_tire_des = steer;  // todo
  steering_controller_.set_steer(state_(IDX::STEER));
  const double new_steer =
    steering_controller_.update_runge_kutta(steer_tire_des, vel, prev_steer_tire_des, dt);
  std::cerr << state_(IDX::STEER) << " -> " << new_steer << std::endl;
  state_(IDX::STEER) =new_steer;
    //   steering_controller_.update_runge_kutta(steer_tire_des, vel, prev_steer_tire_des, dt);
    // steering_controller_.update_euler(steer_tire_des, vel, prev_steer_tire_des, dt);
}

Eigen::VectorXd SimModelActuationCmd::calcLongitudinalModel(
  const Eigen::VectorXd & state, const Eigen::VectorXd & input)
{
  using autoware_vehicle_msgs::msg::GearCommand;

  const double vel = std::clamp(state(IDX::VX), -vx_lim_, vx_lim_);
  const double acc = std::clamp(state(IDX::ACCX), -vx_rate_lim_, vx_rate_lim_);
  const double steer = state(IDX::STEER);
  const double yaw = state(IDX::YAW);
  const double accel = input(IDX_U::ACCEL_DES);
  const double brake = input(IDX_U::BRAKE_DES);
  const auto gear = input(IDX_U::GEAR);

  // calculate acceleration by accel and brake command
  const double acc_des_wo_slope = std::clamp(
    std::invoke([&]() -> double {
      if (convert_accel_cmd_ && accel > 0.0) {
        return accel_map_.getControlCommand(accel, vel);
      } else if (convert_brake_cmd_ && brake > 0.0) {
        return brake_map_.getControlCommand(brake, vel);
      } else {
        return accel;
      }
    }),
    -vx_rate_lim_, vx_rate_lim_);

  // add slope acceleration considering the gear state
  const double acc_by_slope = input(IDX_U::SLOPE_ACCX);
  const double acc_des = std::invoke([&]() -> double {
    if (gear == GearCommand::NONE || gear == GearCommand::PARK) {
      return 0.0;
    } else if (gear == GearCommand::REVERSE || gear == GearCommand::REVERSE_2) {
      return -acc_des_wo_slope + acc_by_slope;
    }
    return acc_des_wo_slope + acc_by_slope;
  });
  const double acc_time_constant = accel > 0.0 ? accel_time_constant_ : brake_time_constant_;

  // calculate derivative of longitudinal states except steering
  Eigen::VectorXd d_longitudinal_state = Eigen::VectorXd::Zero(dim_x_);
  d_longitudinal_state(IDX::X) = vel * cos(yaw);
  d_longitudinal_state(IDX::Y) = vel * sin(yaw);
  d_longitudinal_state(IDX::YAW) = vel * std::tan(steer) / wheelbase_;
  d_longitudinal_state(IDX::VX) = acc;
  d_longitudinal_state(IDX::ACCX) = -(acc - acc_des) / acc_time_constant;

  return d_longitudinal_state;
}

double SimModelActuationCmd::calcLateralModel(
  const double steer, const double steer_input, const double vel)
{
  const double steer_rate = std::clamp(
    std::invoke([&]() -> double {
      if (convert_steer_cmd_) {
        if (actuation_sim_type_ == ActuationSimType::VGR) {
          const double steer_des = calculateSteeringTireCommand(vel, steer, steer_input);
          return -(getSteer() - steer_des) / steer_time_constant_;
        } else if (actuation_sim_type_ == ActuationSimType::STEER_MAP) {
          return steer_map_.getControlCommand(steer_input, vel) / steer_time_constant_;
        }
      }
      const double steer_des = std::clamp(steer_input, -steer_lim_, steer_lim_);
      return -(getSteer() - steer_des) / steer_time_constant_;
    }),
    -steer_rate_lim_, steer_rate_lim_);

  return steer_rate;
}

Eigen::VectorXd SimModelActuationCmd::calcModel(
  const Eigen::VectorXd & state, const Eigen::VectorXd & input)
{
  Eigen::VectorXd d_state = calcLongitudinalModel(state, input);
  d_state(IDX::STEER) =
    calcLateralModel(state(IDX::STEER), input(IDX_U::STEER_DES), state(IDX::VX));
  return d_state;
}

void SimModelActuationCmd::updateStateWithGear(
  Eigen::VectorXd & state, const Eigen::VectorXd & prev_state, const uint8_t gear, const double dt)
{
  using autoware_vehicle_msgs::msg::GearCommand;
  if (
    gear == GearCommand::DRIVE || gear == GearCommand::DRIVE_2 || gear == GearCommand::DRIVE_3 ||
    gear == GearCommand::DRIVE_4 || gear == GearCommand::DRIVE_5 || gear == GearCommand::DRIVE_6 ||
    gear == GearCommand::DRIVE_7 || gear == GearCommand::DRIVE_8 || gear == GearCommand::DRIVE_9 ||
    gear == GearCommand::DRIVE_10 || gear == GearCommand::DRIVE_11 ||
    gear == GearCommand::DRIVE_12 || gear == GearCommand::DRIVE_13 ||
    gear == GearCommand::DRIVE_14 || gear == GearCommand::DRIVE_15 ||
    gear == GearCommand::DRIVE_16 || gear == GearCommand::DRIVE_17 ||
    gear == GearCommand::DRIVE_18 || gear == GearCommand::LOW || gear == GearCommand::LOW_2) {
    if (state(IDX::VX) < 0.0) {
      state(IDX::VX) = 0.0;
      state(IDX::X) = prev_state(IDX::X);
      state(IDX::Y) = prev_state(IDX::Y);
      state(IDX::YAW) = prev_state(IDX::YAW);
      state(IDX::ACCX) = (state(IDX::VX) - prev_state(IDX::VX)) / std::max(dt, 1.0e-5);
    }
  } else if (gear == GearCommand::REVERSE || gear == GearCommand::REVERSE_2) {
    if (state(IDX::VX) > 0.0) {
      state(IDX::VX) = 0.0;
      state(IDX::X) = prev_state(IDX::X);
      state(IDX::Y) = prev_state(IDX::Y);
      state(IDX::YAW) = prev_state(IDX::YAW);
      state(IDX::ACCX) = (state(IDX::VX) - prev_state(IDX::VX)) / std::max(dt, 1.0e-5);
    }
  } else {  // including 'gear == GearCommand::PARK'
    state(IDX::VX) = 0.0;
    state(IDX::X) = prev_state(IDX::X);
    state(IDX::Y) = prev_state(IDX::Y);
    state(IDX::YAW) = prev_state(IDX::YAW);
    state(IDX::ACCX) = (state(IDX::VX) - prev_state(IDX::VX)) / std::max(dt, 1.0e-5);
  }
}

std::optional<tier4_vehicle_msgs::msg::ActuationStatusStamped>
SimModelActuationCmd::getActuationStatus() const
{
  if (!convert_accel_cmd_ && !convert_brake_cmd_ && !convert_steer_cmd_) {
    return std::nullopt;
  }

  tier4_vehicle_msgs::msg::ActuationStatusStamped actuation_status;

  const double acc_state = std::clamp(state_(IDX::ACCX), -vx_rate_lim_, vx_rate_lim_);
  const double vel_state = std::clamp(state_(IDX::VX), -vx_lim_, vx_lim_);

  if (convert_accel_cmd_) {
    const auto throttle = accel_map_.getThrottle(acc_state, vel_state);
    if (throttle.has_value()) {
      actuation_status.status.accel_status = throttle.value();
    }
  }

  if (convert_brake_cmd_) {
    actuation_status.status.brake_status = brake_map_.getBrake(acc_state, vel_state);
  }

  if (convert_steer_cmd_) {
    if (actuation_sim_type_ == ActuationSimType::VGR) {
      actuation_status.status.steer_status =
        calculateSteeringWheelState(vel_state, state_(IDX::STEER));
    }
    // NOTE: Conversion by steer map is not supported
    // else if (actuation_sim_type_ == ActuationSimType::STEER_MAP) {}
  }

  return actuation_status;
}

/* ------ Functions for VGR sim model ----- */
double SimModelActuationCmd::calculateSteeringTireCommand(
  const double vel, const double steer, const double steer_wheel_des) const
{
  // steer_tire_state -> steer_wheel_state
  const double steer_wheel = calculateSteeringWheelState(vel, steer);

  // steer_wheel_des -> steer_tire_des
  const double adaptive_gear_ratio = calculateVariableGearRatio(vel, steer_wheel);

  return steer_wheel_des / adaptive_gear_ratio;
}

double SimModelActuationCmd::calculateSteeringWheelState(
  const double vel, const double steer_state) const
{
  return (vgr_coef_a_ + vgr_coef_b_ * vel * vel) * steer_state /
         (1.0 + vgr_coef_c_ * std::abs(steer_state));
}

double SimModelActuationCmd::calculateVariableGearRatio(
  const double vel, const double steer_wheel) const
{
  return std::max(
    1e-5, vgr_coef_a_ + vgr_coef_b_ * vel * vel - vgr_coef_c_ * std::fabs(steer_wheel));
}
/* ---------------------------------------- */
