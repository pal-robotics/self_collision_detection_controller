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

#ifndef PID_CONTROLLER__PID_CONTROLLER_HPP_
#define PID_CONTROLLER__PID_CONTROLLER_HPP_

#include <memory>
#include <string>
#include <vector>

#include "control_msgs/msg/multi_dof_command.hpp"
#include "control_msgs/msg/multi_dof_state_stamped.hpp"
#include "control_toolbox/pid_ros.hpp"
#include "controller_interface/chainable_controller_interface.hpp"
#include "std_msgs/msg/float64.hpp"
#include "safety_controller_parameters.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "realtime_tools/realtime_buffer.h"
#include "realtime_tools/realtime_publisher.h"
#include "std_srvs/srv/set_bool.hpp"
#include "controller_interface/helpers.hpp"
#include "joint_limits/joint_limits.hpp"
#include "joint_limits/joint_limits_urdf.hpp"
#include "joint_limits/joint_limiter_interface.hpp"
#include "joint_limits/joint_saturation_limiter.hpp"
#include "urdf/model.h"

namespace safety_controller
{
class SafetyController : public controller_interface::ChainableControllerInterface
{
public:
  SafetyController();
  controller_interface::CallbackReturn on_init() override;
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  controller_interface::CallbackReturn on_cleanup(
    const rclcpp_lifecycle::State & previous_state) override;

  controller_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  controller_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  controller_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  controller_interface::return_type update_reference_from_subscribers(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  controller_interface::return_type update_and_write_commands(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  using ControllerReferenceMsg = control_msgs::msg::MultiDOFCommand;
  using ControllerStateMsg = control_msgs::msg::MultiDOFStateStamped;

protected:
  std::shared_ptr<safety_controller::ParamListener> param_listener_;
  safety_controller::Params params_;

  std::vector<std::string> reference_and_state_dof_names_;
  size_t dof_;
  std::vector<double> measured_state_values_;

  // Feed-forward velocity weight factor when calculating closed loop pid adapter's command
  std::vector<double> feedforward_gain_;

  std::map<std::string, int> joint_id_;

  // Command subscribers and Controller State publisher
  rclcpp::Subscription<ControllerReferenceMsg>::SharedPtr ref_subscriber_ = nullptr;
  realtime_tools::RealtimeBuffer<std::shared_ptr<ControllerReferenceMsg>> input_ref_;

  using ControllerStatePublisher = realtime_tools::RealtimePublisher<ControllerStateMsg>;

  rclcpp::Publisher<ControllerStateMsg>::SharedPtr s_publisher_;
  std::unique_ptr<ControllerStatePublisher> state_publisher_;

  // override methods from ChainableControllerInterface
  std::vector<hardware_interface::CommandInterface> on_export_reference_interfaces() override;

  std::vector<hardware_interface::StateInterface> on_export_state_interfaces() override;

  bool on_set_chained_mode(bool chained_mode) override;

  // internal methods
  void update_parameters();
  controller_interface::CallbackReturn configure_parameters();

private:
  // callback for topic interface
  std::vector<joint_limits::JointLimits> hard_limits;
  std::vector<joint_limits::SoftJointLimits> soft_limits;
  std::vector<double> command_violated_;

  bool limit_violated_prev_ = false;
  std::vector<double> index_violated_prev_;
  std::vector<double> current_position_, current_reference_;
  void reference_callback(const std::shared_ptr<ControllerReferenceMsg> msg);
};

}  // namespace pid_controller

#endif  // PID_CONTROLLER__PID_CONTROLLER_HPP_
