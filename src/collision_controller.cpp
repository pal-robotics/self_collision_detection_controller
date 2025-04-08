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

#include "collision_controller/collision_controller.hpp"

#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "angles/angles.h"
#include "control_msgs/msg/single_dof_state.hpp"
#include "rclcpp/version.h"
#include "rclcpp/rclcpp.hpp"
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/compute-all-terms.hpp>
#include <pinocchio/algorithm/model.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include "pinocchio/parsers/urdf.hpp"
#include "pinocchio/parsers/srdf.hpp"
#include "change_controllers_interfaces/action/switch.hpp"
#include <chrono>

namespace
{


using ControllerCommandMsg = collision_controller::CollisionController::ControllerReferenceMsg;
// called from RT control loop
void reset_controller_reference_msg(
  const std::shared_ptr<ControllerCommandMsg> & msg, const std::vector<std::string> & dof_names)
{
  msg->dof_names.resize(dof_names.size());
  msg->dof_names = dof_names;
  msg->values.resize(dof_names.size(), std::numeric_limits<double>::quiet_NaN());
  msg->values_dot.resize(dof_names.size(), std::numeric_limits<double>::quiet_NaN());
}
}  // namespace

namespace collision_controller
{

using std::placeholders::_1;

CollisionController::CollisionController()
: controller_interface::ChainableControllerInterface(),
  input_ref_(nullptr)
{
}

controller_interface::CallbackReturn CollisionController::on_init()
{

  try {
    param_listener_ = std::make_shared<collision_controller::ParamListener>(get_node());
    params_ = param_listener_->get_params();
  } catch (const std::exception & e) {
    fprintf(stderr, "Exception thrown during controller's init with message: %s \n", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

void CollisionController::update_parameters()
{
  if (!param_listener_->is_old(params_)) {
    return;
  }
  params_ = param_listener_->get_params();
}

controller_interface::CallbackReturn CollisionController::on_cleanup(
  const rclcpp_lifecycle::State & /*previous_state*/)
{

  return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn CollisionController::on_configure(
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

  command_interfaces_.reserve(1);
  state_interfaces_.reserve(params_.joints.size());
  reference_interfaces_.resize(params_.commanded_joints.size());
  current_position_.resize(params_.joints.size());
  current_reference_.resize(params_.commanded_joints.size());
  command_violated_.resize(params_.commanded_joints.size());

  // topics QoS
  auto subscribers_qos = rclcpp::SystemDefaultsQoS();
  subscribers_qos.keep_last(1);
  subscribers_qos.best_effort();

  // Change controllers client
  change_controller_client_ =
    rclcpp_action::create_client<change_controllers_interfaces::action::Switch>(
    this->get_node(),
    "change_controllers");

  // Reference Subscriber
  ref_subscriber_ = get_node()->create_subscription<ControllerReferenceMsg>(
    "~/reference", subscribers_qos,
    std::bind(&CollisionController::reference_callback, this, std::placeholders::_1));

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

    ++idx;
  }

  idx = 0;
  for (const auto & joint : params_.commanded_joints) {
    auto joint_name = joint;

    joint_id_commanded_.insert(std::make_pair(joint, idx));

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


  // Creating pinocchio model for collision handling

  pinocchio::urdf::buildModelFromXML(
    this->get_robot_description(),
    pinocchio::JointModelFreeFlyer(), model_);

  aux_node_ = std::make_shared<rclcpp::Node>("aux_node");
  robot_desc_sub_ = aux_node_->create_subscription<std_msgs::msg::String>(
    "/robot_description_semantic", rclcpp::QoS(1).transient_local(),
    [&](const std::shared_ptr<std_msgs::msg::String> msg) -> void
    {
      srdf_model = msg->data;
      RCLCPP_INFO_ONCE(get_node()->get_logger(), "Received robot description");
    });

  while (srdf_model.empty()) {

    rclcpp::spin_some(aux_node_);
    std::this_thread::sleep_for(std::chrono_literals::operator""ms(100));
    RCLCPP_INFO_THROTTLE(
      get_node()->get_logger(),
      *get_node()->get_clock(), 1000, "Waiting for the robot semantic description!");
  }


  std::vector<pinocchio::JointIndex> joints_to_lock;
  /*VMO: In this point we make a list of joints to remove from the model (in tiago case wheel joints and end effector joints)
   These are actually the joints that we don't passs to the controller*/
  for (auto & name : model_.names) {
    if (name != "universe" && name != "root_joint" &&
      std::find(
        params_.joints.begin(), params_.joints.end(),
        name) == params_.joints.end())
    {
      joints_to_lock.push_back(model_.getJointId(name));
      RCLCPP_INFO(get_node()->get_logger(), "Lock joint %s: ", name.c_str());
    }
  }

  /* Removing the unused joints from the model*/
  model_ =
    pinocchio::buildReducedModel(model_, joints_to_lock, pinocchio::neutral(model_));

  data_ = pinocchio::Data(model_);

  // Taking current description of the robot
  std::istringstream ss(this->get_robot_description());
  pinocchio::urdf::buildGeom(
    model_,
    ss, pinocchio::COLLISION, geom_model_);
  geom_model_.addAllCollisionPairs();

  update_parameters();

  pinocchio::srdf::removeCollisionPairsFromXML(model_, geom_model_, srdf_model);

  geom_data_ = pinocchio::GeometryData(geom_model_);


  pinocchio::GeometryData::MatrixXs security_margin_map(pinocchio::GeometryData::MatrixXs::
    Ones(
      (Eigen::DenseIndex)geom_model_.ngeoms, (Eigen::DenseIndex)geom_model_.ngeoms));
  security_margin_map.triangularView<Eigen::Upper>().fill(0.05);
  security_margin_map.triangularView<Eigen::Lower>().fill(0.);

  pinocchio::GeometryData::MatrixXs security_margin_map_upper(security_margin_map);
  security_margin_map_upper.triangularView<Eigen::Lower>().fill(0.);

  geom_data_.setSecurityMargins(geom_model_, security_margin_map, true, true);


  return controller_interface::CallbackReturn::SUCCESS;
}

void CollisionController::reference_callback(const std::shared_ptr<ControllerReferenceMsg> msg)
{
  if (msg->dof_names.empty() && msg->values.size() == params_.commanded_joints.size()) {
    RCLCPP_WARN(
      get_node()->get_logger(),
      "Reference massage does not have DoF names defined. "
      "Assuming that value have order as defined state DoFs");
    auto ref_msg = msg;
    ref_msg->dof_names = params_.commanded_joints;
    input_ref_.writeFromNonRT(ref_msg);
  } else if (
    msg->dof_names.size() == params_.commanded_joints.size() &&
    msg->values.size() == params_.joints.size())
  {
    auto ref_msg = msg;   // simple initialization

    // sort values in the ref_msg
    reset_controller_reference_msg(msg, params_.commanded_joints);

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
      msg->dof_names.size(), msg->values.size(), params_.commanded_joints.size());
  }


}

controller_interface::InterfaceConfiguration CollisionController::command_interface_configuration()
const
{
  controller_interface::InterfaceConfiguration command_interfaces_config;
  command_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  command_interfaces_config.names.reserve(1);
  for (const auto & dof_name : params_.commanded_joints) {
    command_interfaces_config.names.push_back("robot/is_in_collision");
  }

  return command_interfaces_config;
}

controller_interface::InterfaceConfiguration CollisionController::state_interface_configuration()
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

std::vector<hardware_interface::CommandInterface> CollisionController::
on_export_reference_interfaces()
{
  //reference_interfaces_.resize(
  //  params_.joints.size(), std::numeric_limits<double>::quiet_NaN());
//
//std::vector<hardware_interface::CommandInterface> reference_interfaces;
//reference_interfaces.reserve(reference_interfaces_.size());
//
//size_t index = 0;
//for (const auto & dof_name : params_.commanded_joints) {
//  reference_interfaces.push_back(
//    hardware_interface::CommandInterface(
//      get_node()->get_name(), dof_name + "/" + params_.command_interface,
//      &reference_interfaces_[index]));
//  ++index;
//}

  reference_interfaces_.clear();

  return {};
}

std::vector<hardware_interface::StateInterface> CollisionController::on_export_state_interfaces()
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

bool CollisionController::on_set_chained_mode(bool chained_mode)
{
  // Always accept switch to/from chained mode
  return true || chained_mode;
}

controller_interface::CallbackReturn CollisionController::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // Set default value in command (the same number as state interfaces)
  //reset_controller_reference_msg(*(input_ref_.readFromRT()), params_.joints);

  // check if action server is available
  if (!change_controller_client_->wait_for_action_server(std::chrono::seconds(1))) {
    RCLCPP_ERROR(get_node()->get_logger(), "Change controller service is not available");
    return controller_interface::CallbackReturn::ERROR;
  }


  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn CollisionController::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type CollisionController::update_reference_from_subscribers(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{
  auto current_ref = input_ref_.readFromRT();


  for (size_t i = 0; i < params_.commanded_joints.size(); ++i) {
    if (!std::isnan((*current_ref)->values[i])) {
      reference_interfaces_[i] = (*current_ref)->values[i];

      (*current_ref)->values[i] = std::numeric_limits<double>::quiet_NaN();
    }
  }

  return controller_interface::return_type::OK;
}

controller_interface::return_type CollisionController::update_and_write_commands(
  const rclcpp::Time & time, const rclcpp::Duration & period)
{
  // check for any parameter updates
  //update_parameters();
  auto start = std::chrono::high_resolution_clock::now();

  Eigen::VectorXd q = Eigen::VectorXd::Zero(params_.joints.size() + 7);

  q[6] = 1.0;
  for (const auto & joint : params_.joints) {
    q.tail(params_.joints.size())[model_.getJointId(joint) -
      2] = state_interfaces_[joint_id_[joint]].get_value();
  }
  //auto end = std::chrono::high_resolution_clock::now();

  //auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
//  RCLCPP_INFO(get_node()->get_logger(), "Execution time: %ld microseconds", duration.count());


  /*for (const auto & joint: params_.commanded_joints) {
    auto idx = joint_id_commanded_.at(joint);
    if (isnan(reference_interfaces_[idx])) {
      RCLCPP_WARN(
        get_node()->get_logger(), "Reference value for joint %s is NaN.", joint.c_str());
      return controller_interface::return_type::OK;
    }
  }*/


  if (!collision_prev) {

    collision_prev = pinocchio::computeCollisions(
      model_, data_,
      geom_model_, geom_data_, q, true);
    // auto end = std::chrono::high_resolution_clock::now();

    //auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
    //RCLCPP_INFO(get_node()->get_logger(), "Execution time: %ld microseconds", duration.count());

    //Checking if there is a collision
    if (collision_prev) {

      command_interfaces_[0].set_value(1.0);

      for (size_t k = 0; k < geom_model_.collisionPairs.size(); ++k) {
        auto cp = geom_model_.collisionPairs[k];
        const hpp::fcl::CollisionResult & cr = geom_data_.collisionResults[k];
        if (cr.isCollision()) {

          RCLCPP_WARN(
            get_node()->get_logger(), "Collision detected, between %s and %s",
            geom_model_.geometryObjects[cp.first].name.c_str(),
            geom_model_.geometryObjects[cp.second].name.c_str());

        }
      }
    } else if (!collision_prev) {

      command_interfaces_[0].set_value(0.0);
    }
  } else {

    collision_prev = pinocchio::computeCollisions(
      model_, data_,
      geom_model_, geom_data_, q, true);

    if (collision_prev) {

      command_interfaces_[0].set_value(1.0);
      for (size_t k = 0; k < geom_model_.collisionPairs.size(); ++k) {
        auto cp = geom_model_.collisionPairs[k];
        const hpp::fcl::CollisionResult & cr = geom_data_.collisionResults[k];

        if (cr.isCollision()) {
          RCLCPP_WARN_THROTTLE(
            get_node()->get_logger(),
            *get_node()->get_clock(),
            1000,
            "Collision detected, between %s and %s",
            geom_model_.geometryObjects[cp.first].name.c_str(),
            geom_model_.geometryObjects[cp.second].name.c_str());
        }
      }
    } else if (!collision_prev) {

      command_interfaces_[0].set_value(0.0);

      RCLCPP_WARN(
        get_node()->get_logger(),
        "Collision has been resolved");
      collision_prev = false;
    }


  }

  auto end = std::chrono::high_resolution_clock::now();

  auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
  RCLCPP_INFO(get_node()->get_logger(), "Execution time: %ld microseconds", duration.count());

  return controller_interface::return_type::OK;
}


void CollisionController::send_goal()
{
  using namespace std::placeholders;

  auto goal_msg = change_controllers_interfaces::action::Switch::Goal();
  goal_msg.start_controllers = {"gravity_compensation_controller"};
  goal_msg.stop_controllers = {};
  goal_msg.switch_controllers = true;
  goal_msg.load = false;
  goal_msg.unload = false;
  goal_msg.configure = false;


  auto send_goal_options =
    rclcpp_action::Client<change_controllers_interfaces::action::Switch>::SendGoalOptions();

  send_goal_options.result_callback =
    std::bind(&CollisionController::result_cb, this, _1);
  change_controller_client_->async_send_goal(goal_msg, send_goal_options);
}

void CollisionController::result_cb(
  const rclcpp_action::ClientGoalHandle<change_controllers_interfaces::action::Switch>::WrappedResult & result)
{
  switch (result.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      return;
    case rclcpp_action::ResultCode::ABORTED:
      RCLCPP_ERROR(get_node()->get_logger(), "Change controllers: Aborted");
      return;
    case rclcpp_action::ResultCode::CANCELED:
      RCLCPP_ERROR(get_node()->get_logger(), "Change controllers: Canceled");
      return;
    default:
      RCLCPP_ERROR(get_node()->get_logger(), "Change controllers: Unknown result code");
      return;
  }
}

}   // namespace collision_controller

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  collision_controller::CollisionController, controller_interface::ChainableControllerInterface)
