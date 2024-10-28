// Copyright (c) 2023, Stogl Robotics Consulting UG (haftungsbeschränkt)
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
//
// Authors: Daniel Azanov, Dr. Denis
//

#include "safety_controller/safety_controller.hpp"

#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "angles/angles.h"
#include "control_msgs/msg/single_dof_state.hpp"
#include "rclcpp/version.h"

namespace
{

using ControllerCommandMsg = safety_controller::SafetyController::ControllerReferenceMsg;
// called from RT control loop
void reset_controller_reference_msg(
  const std::shared_ptr<ControllerCommandMsg> & msg, const std::vector<std::string> & dof_names)
{
  msg->dof_names.resize(dof_names.size());
  msg->dof_names = dof_names;
  msg->values.resize(dof_names.size(), std::numeric_limits<double>::quiet_NaN());
  msg->values_dot.resize(dof_names.size(), std::numeric_limits<double>::quiet_NaN());
}

void reset_controller_measured_state_msg(
  const std::shared_ptr<ControllerCommandMsg> & msg, const std::vector<std::string> & dof_names)
{
  reset_controller_reference_msg(msg, dof_names);
}

}  // namespace

namespace safety_controller
{
SafetyController::SafetyController()
: controller_interface::ChainableControllerInterface(),
  input_ref_(nullptr)
{
}

controller_interface::CallbackReturn SafetyController::on_init()
{

  try {
    param_listener_ = std::make_shared<safety_controller::ParamListener>(get_node());
    params_ = param_listener_->get_params();
  } catch (const std::exception & e) {
    fprintf(stderr, "Exception thrown during controller's init with message: %s \n", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

void SafetyController::update_parameters()
{
  if (!param_listener_->is_old(params_)) {
    return;
  }
  params_ = param_listener_->get_params();
}

controller_interface::CallbackReturn SafetyController::on_cleanup(
  const rclcpp_lifecycle::State & /*previous_state*/)
{

  return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn SafetyController::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  if (!param_listener_) {
    RCLCPP_ERROR(get_node()->get_logger(), "Error encountered during init");
    return controller_interface::CallbackReturn::ERROR;
  }

  update_parameters();

  if (params_.joints.empty()) {
    RCLCPP_ERROR(get_node()->get_logger(), "No joints have been passed to the controller.");
    return controller_interface::CallbackReturn::ERROR;
  }

  if (params_.command_interface.empty()) {
    RCLCPP_ERROR(get_node()->get_logger(), "'command_interfaces' parameter was empty");
    return controller_interface::CallbackReturn::ERROR;
  }

  command_interfaces_.reserve(params_.joints.size());
  state_interfaces_.reserve(params_.joints.size());
  reference_interfaces_.resize(params_.joints.size());
  current_position_.resize(params_.joints.size());
  current_reference_.resize(params_.joints.size());
  command_violated_.resize(params_.joints.size());

  // topics QoS
  auto subscribers_qos = rclcpp::SystemDefaultsQoS();
  subscribers_qos.keep_last(1);
  subscribers_qos.best_effort();

  // Reference Subscriber
  ref_subscriber_ = get_node()->create_subscription<ControllerReferenceMsg>(
    "~/reference", subscribers_qos,
    std::bind(&SafetyController::reference_callback, this, std::placeholders::_1));

  try {
    // State publisher
    s_publisher_ = get_node()->create_publisher<ControllerStateMsg>(
      "~/controller_state", rclcpp::SystemDefaultsQoS());
    state_publisher_ = std::make_unique<ControllerStatePublisher>(s_publisher_);
  } catch (const std::exception & e) {
    fprintf(
      stderr, "Exception thrown during publisher creation at configure stage with message : %s \n",
      e.what());
    return controller_interface::CallbackReturn::ERROR;
  }

  // Reserve memory in state publisher
  state_publisher_->lock();
  state_publisher_->msg_.dof_states.resize(params_.joints.size());
  for (size_t i = 0; i < params_.joints.size(); ++i) {
    state_publisher_->msg_.dof_states[i].name = params_.joints[i];
  }
  state_publisher_->unlock();

  const std::string urdf = get_robot_description();

  urdf::Model model;
  if (!model.initString(urdf)) {
    RCLCPP_ERROR(get_node()->get_logger(), "Failed to parse URDF");
    return controller_interface::CallbackReturn::ERROR;
  }

  std::shared_ptr<ControllerReferenceMsg> msg = std::make_shared<ControllerReferenceMsg>();
  reset_controller_reference_msg(msg, params_.joints);
  //input_ref_.writeFromNonRT(msg);

  hard_limits.resize(params_.joints.size());
  soft_limits.resize(params_.joints.size());

  int idx = 0;
  for (const auto & joint : params_.joints) {
    auto joint_name = joint;

    joint_id_.insert(std::make_pair(joint, idx));

    if (!joint_limits::getSoftJointLimits(model.getJoint(joint_name), soft_limits[idx])) {
      RCLCPP_ERROR(get_node()->get_logger(), "Failed to get hard limits from URDF");
      return controller_interface::CallbackReturn::ERROR;
    }

    if (!joint_limits::getJointLimits(model.getJoint(joint_name), hard_limits[idx])) {
      RCLCPP_ERROR(get_node()->get_logger(), "Failed to get hard limits from URDF");
      return controller_interface::CallbackReturn::ERROR;
    }

    ++idx;
  }

  RCLCPP_INFO(get_node()->get_logger(), "configure successful");
  return controller_interface::CallbackReturn::SUCCESS;
}

void SafetyController::reference_callback(const std::shared_ptr<ControllerReferenceMsg> msg)
{
  if (msg->dof_names.empty() && msg->values.size() == params_.joints.size()) {
    RCLCPP_WARN(
      get_node()->get_logger(),
      "Reference massage does not have DoF names defined. "
      "Assuming that value have order as defined state DoFs");
    auto ref_msg = msg;
    ref_msg->dof_names = params_.joints;
    input_ref_.writeFromNonRT(ref_msg);
  } else if (
    msg->dof_names.size() == params_.joints.size() &&
    msg->values.size() == params_.joints.size())
  {
    auto ref_msg = msg;   // simple initialization

    // sort values in the ref_msg
    reset_controller_reference_msg(msg, params_.joints);

    bool all_found = true;
    for (size_t i = 0; i < msg->dof_names.size(); ++i) {
      auto found_it =
        std::find(ref_msg->dof_names.begin(), ref_msg->dof_names.end(), msg->dof_names[i]);
      if (found_it == msg->dof_names.end()) {
        all_found = false;
        RCLCPP_WARN(
          get_node()->get_logger(), "DoF name '%s' not found in the defined list of state DoFs.",
          msg->dof_names[i].c_str());
        break;
      }

      auto position = std::distance(ref_msg->dof_names.begin(), found_it);
      ref_msg->values[position] = msg->values[i];
    }
    if (all_found) {
      input_ref_.writeFromNonRT(ref_msg);
    }
  } else {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "Size of input data names (%zu) and/or values (%zu) is not matching the expected size (%zu).",
      msg->dof_names.size(), msg->values.size(), params_.joints.size());
  }


}

controller_interface::InterfaceConfiguration SafetyController::command_interface_configuration()
const
{
  controller_interface::InterfaceConfiguration command_interfaces_config;
  command_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  command_interfaces_config.names.reserve(params_.joints.size());
  for (const auto & dof_name : params_.joints) {
    command_interfaces_config.names.push_back(dof_name + "/" + params_.command_interface);
  }

  return command_interfaces_config;
}

controller_interface::InterfaceConfiguration SafetyController::state_interface_configuration()
const
{
  controller_interface::InterfaceConfiguration state_interfaces_config;

  state_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  state_interfaces_config.names.reserve(params_.joints.size());
  for (const auto & dof_name : params_.joints) {
    state_interfaces_config.names.push_back(dof_name + "/position");
  }


  return state_interfaces_config;
}

std::vector<hardware_interface::CommandInterface> SafetyController::on_export_reference_interfaces()
{
  reference_interfaces_.resize(
    params_.joints.size(), std::numeric_limits<double>::quiet_NaN());

  std::vector<hardware_interface::CommandInterface> reference_interfaces;
  reference_interfaces.reserve(reference_interfaces_.size());

  size_t index = 0;
  for (const auto & dof_name : params_.joints) {
    reference_interfaces.push_back(
      hardware_interface::CommandInterface(
        get_node()->get_name(), dof_name + "/" + params_.command_interface,
        &reference_interfaces_[index]));
    ++index;
  }


  return reference_interfaces;
}

std::vector<hardware_interface::StateInterface> SafetyController::on_export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  state_interfaces.reserve(params_.joints.size());

  state_interfaces_values_.resize(
    params_.joints.size(),
    std::numeric_limits<double>::quiet_NaN());
  size_t index = 0;
  for (const auto & dof_name : params_.joints) {
    state_interfaces.push_back(
      hardware_interface::StateInterface(
        get_node()->get_name(), dof_name + "/" + params_.command_interface,
        &state_interfaces_values_[index]));
    ++index;
  }

  return state_interfaces;
}

bool SafetyController::on_set_chained_mode(bool chained_mode)
{
  // Always accept switch to/from chained mode
  return true || chained_mode;
}

controller_interface::CallbackReturn SafetyController::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // Set default value in command (the same number as state interfaces)
  //reset_controller_reference_msg(*(input_ref_.readFromRT()), params_.joints);


  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn SafetyController::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type SafetyController::update_reference_from_subscribers(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{
  auto current_ref = input_ref_.readFromRT();


  for (size_t i = 0; i < dof_; ++i) {
    if (!std::isnan((*current_ref)->values[i])) {
      reference_interfaces_[i] = (*current_ref)->values[i];
      if (reference_interfaces_.size() == 2 * dof_ &&
        !std::isnan((*current_ref)->values_dot[i]))
      {
        reference_interfaces_[dof_ + i] = (*current_ref)->values_dot[i];
      }

      (*current_ref)->values[i] = std::numeric_limits<double>::quiet_NaN();
    }
  }
  return controller_interface::return_type::OK;
}

controller_interface::return_type SafetyController::update_and_write_commands(
  const rclcpp::Time & time, const rclcpp::Duration & period)
{
  // check for any parameter updates
  update_parameters();


  // no command received yet
  std::vector<int> index_violated;
  bool limit_violated = false;


  for (const auto joint: params_.joints) {
    auto idx = joint_id_.at(joint);
    current_position_[idx] = state_interfaces_[idx].get_value();
    current_reference_[idx] = reference_interfaces_[idx];

    if (isnan(reference_interfaces_[idx])) {
      RCLCPP_WARN(
        get_node()->get_logger(), "Reference value for joint %s is NaN.", joint.c_str());
      return controller_interface::return_type::OK;
    }

    // check if the reference is within the limits
    auto pos = std::clamp(
      current_reference_[idx], soft_limits[idx].min_position, soft_limits[idx].max_position);
    if (pos != current_reference_[idx]) {
      if (std::find(
          index_violated_prev_.begin(), index_violated_prev_.end(),
          idx) == index_violated_prev_.end())
      {
        RCLCPP_WARN(
          get_node()->get_logger(), "Reference value for joint %s is out of soft limits.",
          joint.c_str());
        RCLCPP_WARN(
          get_node()->get_logger(), "Reference value clamped to %f.",
          pos);
        index_violated_prev_.push_back(idx);
      }
      // set the reference to the clamped value
      limit_violated = true;
      current_reference_[idx] = pos;
      index_violated.push_back(idx);

    }
  }

  // If the limit is not anymore violated, reset the flag
  if (!limit_violated) {
    if (limit_violated_prev_) {
      RCLCPP_WARN(
        get_node()->get_logger(), "Reference values have been brought back within the limits.");
    }
    limit_violated_prev_ = false;
    index_violated_prev_.clear();
  }

  // If the limit is still violated, set the command to the last valid value
  if (limit_violated && limit_violated_prev_) {
    for (const auto joint: params_.joints) {
      auto idx = joint_id_.at(joint);
      command_interfaces_[idx].set_value(command_violated_[idx]);
    }
  }
  // If the limit is violated but it wasn't before, set the command to the current position
  else if (limit_violated && !limit_violated_prev_) {
    for (const auto joint: params_.joints) {
      auto idx = joint_id_.at(joint);
      if (std::find(index_violated.begin(), index_violated.end(), idx) == index_violated.end()) {
        command_interfaces_[idx].set_value(current_position_[idx]);
      } else {
        // The joint violating the limits is set to the clamped reference
        command_interfaces_[idx].set_value(current_reference_[idx]);
      }
      //Saving the commands for next iterations
      command_violated_[idx] = command_interfaces_[idx].get_value();
      limit_violated_prev_ = true;
    }
  } else {
    // If no limit is violated, set the command to the reference
    for (const auto joint: params_.joints) {

      auto idx = joint_id_.at(joint);
      command_interfaces_[idx].set_value(current_reference_[idx]);
    }
  }

  // Fill the information of the exported state interfaces

  if (state_publisher_ && state_publisher_->trylock()) {
    state_publisher_->msg_.header.stamp = time;
    for (size_t i = 0; i < params_.joints.size(); ++i) {
      state_publisher_->msg_.dof_states[i].reference = reference_interfaces_[i];
      state_publisher_->msg_.dof_states[i].feedback = current_position_[i];
      state_publisher_->msg_.dof_states[i].time_step = period.seconds();
      // Command can store the old calculated values. This should be obvious because at least one
      // another value is NaN.
      state_publisher_->msg_.dof_states[i].output = command_interfaces_[i].get_value();
    }
    state_publisher_->unlockAndPublish();
  }

  return controller_interface::return_type::OK;
}

}  // namespace safety_controller

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  safety_controller::SafetyController, controller_interface::ChainableControllerInterface)
