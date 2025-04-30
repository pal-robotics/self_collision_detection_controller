// Copyright (c) 2024 PAL Robotics S.L. All rights reserved.
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

#ifndef COLLISION_CONTROLLER_COLLISION_CONTROLLER_HPP_
#define COLLISION_CONTROLLER__COLLISION_CONTROLLER_HPP_

#include <memory>
#include <string>
#include <vector>

#include "control_msgs/msg/multi_dof_command.hpp"
#include "control_msgs/msg/multi_dof_state_stamped.hpp"
#include "controller_interface/chainable_controller_interface.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/string.hpp"
#include "collision_controller/collision_controller_parameters.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include <realtime_tools/realtime_buffer.hpp>
#include <realtime_tools/realtime_publisher.hpp>
#include "std_srvs/srv/set_bool.hpp"
#include "controller_interface/helpers.hpp"
#include "joint_limits/joint_limits.hpp"
#include "joint_limits/joint_limits_urdf.hpp"
#include "joint_limits/joint_limiter_interface.hpp"
#include "joint_limits/joint_saturation_limiter.hpp"
#include "urdf/model.h"
#include <pinocchio/parsers/urdf.hpp>

#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/multibody/geometry.hpp"

#include "pinocchio/multibody/model.hpp"
#include "pinocchio/multibody/data.hpp"
#include "pinocchio/multibody/geometry.hpp"
#include "change_controllers_interfaces/action/switch.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include <pinocchio/collision/collision.hpp>
#include <pinocchio/fwd.hpp>
#include "pinocchio/parsers/srdf.hpp"
#include <visualization_msgs/msg/marker.hpp>


namespace collision_controller
{
class CollisionController : public controller_interface::ChainableControllerInterface
{
public:
  CollisionController();
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
  std::shared_ptr<collision_controller::ParamListener> param_listener_;
  collision_controller::Params params_;

  std::map<std::string, int> joint_id_;
  std::map<std::string, int> joint_id_commanded_;

  // Command subscribers and Controller State publisher
  rclcpp::Subscription<ControllerReferenceMsg>::SharedPtr ref_subscriber_ = nullptr;
  realtime_tools::RealtimeBuffer<std::shared_ptr<ControllerReferenceMsg>> input_ref_;

  using ControllerStatePublisher = realtime_tools::RealtimePublisher<ControllerStateMsg>;

  rclcpp::Publisher<ControllerStateMsg>::SharedPtr s_publisher_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr robot_desc_sub_;

  std::unique_ptr<ControllerStatePublisher> state_publisher_;

  // override methods from ChainableControllerInterface
  std::vector<hardware_interface::CommandInterface> on_export_reference_interfaces() override;

  std::vector<hardware_interface::StateInterface> on_export_state_interfaces() override;

  void publish_collision_meshes();


  bool on_set_chained_mode(bool chained_mode) override;

  // internal methods
  void update_parameters();
  controller_interface::CallbackReturn configure_parameters();

  void removeCollisionObjectsForLinks(const std::vector<std::string> & link_names);
  void removeCollisionsAndAddSphere(
    const std::vector<std::string> & to_remove_names, double radius,
    std::string name_collision);

  void removeCollisionBetweenLinks(const std::string & link1, const std::string & link2);

private:
  // callback for topic interface
  std::vector<joint_limits::JointLimits> hard_limits;
  std::vector<joint_limits::SoftJointLimits> soft_limits;
  std::vector<double> command_violated_;

  bool limit_violated_prev_ = false;
  std::vector<double> index_violated_prev_;
  std::vector<double> current_position_, current_reference_;
  void reference_callback(const std::shared_ptr<ControllerReferenceMsg> msg);
  rclcpp::Node::SharedPtr aux_node_ = nullptr;
  pinocchio::Model model_;
  pinocchio::GeometryModel geom_model_;

  pinocchio::Data data_;
  pinocchio::GeometryData geom_data_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
  std::string srdf_model;

  bool collision_prev = false;
  size_t collision_pair = 0;

  rclcpp_action::Client<change_controllers_interfaces::action::Switch>::SharedPtr
    change_controller_client_;
};

}
#endif
