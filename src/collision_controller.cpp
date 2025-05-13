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
#include <visualization_msgs/msg/marker.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
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

  marker_pub_ =
    get_node()->create_publisher<visualization_msgs::msg::Marker>("collision_meshes", 10);
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
    [&](const std::shared_ptr<std_msgs::msg::String> msg_semantic) -> void
    {
      srdf_model = msg_semantic->data;
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

  std::vector<std::string> to_remove_names = {
    "base_front",
    "base_rear",
    "base_imu",
    "base_link_1",
    "base_link_2",
    "base_antenna",
    "base_dock",
    "wheel",
    "suspension",
    "torso_head",
    "torso_fixed"
  };
  removeCollisionObjectsForLinks(to_remove_names);

  std::vector<std::string> gripper_head = {
    "gripper_head"
  };
  removeCollisionsAndAddSphere(
    gripper_head, 0.05, "gripper_head");

  std::vector<std::string> gripper_left;
  gripper_left.push_back("gripper_left");
  removeCollisionsAndAddSphere(
    gripper_left, 0.05, "gripper_left");

  std::vector<std::string> gripper_right;
  gripper_right.push_back("gripper_right");
  removeCollisionsAndAddSphere(
    gripper_right, 0.05, "gripper_right");


  geom_model_.collisionPairs.clear();

  // Rebuild all valid collision pairs
  geom_model_.addAllCollisionPairs();

  pinocchio::srdf::removeCollisionPairsFromXML(model_, geom_model_, srdf_model);

  removeCollisionBetweenLinks(
    "gripper_head", "arm_head_6_link_0");

  removeCollisionBetweenLinks(
    "gripper_left", "arm_left_6_link_0");
  removeCollisionBetweenLinks(
    "gripper_right", "arm_right_6_link_0");

  removeCollisionBetweenLinks(
    "gripper_head", "arm_head_5_link_0");

  removeCollisionBetweenLinks(
    "gripper_left", "arm_left_5_link_0");
  removeCollisionBetweenLinks(
    "gripper_right", "arm_right_5_link_0");
  geom_data_ = pinocchio::GeometryData(geom_model_);


  for (std::size_t i = 0; i < geom_model_.collisionPairs.size(); ++i) {
    const pinocchio::CollisionPair & pair = geom_model_.collisionPairs[i];

    const std::string & name1 = geom_model_.geometryObjects[pair.first].name;
    const std::string & name2 = geom_model_.geometryObjects[pair.second].name;

    std::cout << "Pair " << i << ": " << name1 << " <--> " << name2 << std::endl;
  }
// Separate high-priority and low-priority pairs
  std::vector<pinocchio::CollisionPair> high_priority_pairs;
  std::vector<pinocchio::CollisionPair> low_priority_pairs;

  for (const auto & pair : geom_model_.collisionPairs) {
    const std::string & name1 = geom_model_.geometryObjects[pair.first].name;
    const std::string & name2 = geom_model_.geometryObjects[pair.second].name;

    if (name1.find("arm") != std::string::npos && name2.find("arm") != std::string::npos) {
      high_priority_pairs.push_back(pair);
    } else {
      low_priority_pairs.push_back(pair);
    }
  }

// Reassign the reordered vector back
  geom_model_.collisionPairs.clear();
  geom_model_.collisionPairs.insert(
    geom_model_.collisionPairs.end(),
    high_priority_pairs.begin(),
    high_priority_pairs.end()
  );
  geom_model_.collisionPairs.insert(
    geom_model_.collisionPairs.end(),
    low_priority_pairs.begin(),
    low_priority_pairs.end()
  );

  using MatrixXs = pinocchio::GeometryData::MatrixXs;

// Initialize the default security margin map
  MatrixXs security_margin_map = MatrixXs::Ones(
    static_cast<Eigen::DenseIndex>(geom_model_.ngeoms),
    static_cast<Eigen::DenseIndex>(geom_model_.ngeoms)
  );

// Fill with default margin
  security_margin_map.triangularView<Eigen::Upper>().setConstant(0.01);
  security_margin_map.triangularView<Eigen::Lower>().setZero();

// Now override pairs where both geometry names contain "arm"
  for (const auto & pair : geom_model_.collisionPairs) {
    const auto & name1 = geom_model_.geometryObjects[pair.first].name;
    const auto & name2 = geom_model_.geometryObjects[pair.second].name;

    if (name1.find("arm") != std::string::npos && name2.find("arm") != std::string::npos) {
      // Set higher margin for "arm"-"arm" pairs
      security_margin_map(pair.first, pair.second) = 0.025;
      // Optional: if symmetric margins are needed, set the reverse as well
    }
  }

// Optionally make upper triangle only (depending on API needs)
  MatrixXs security_margin_map_upper = security_margin_map;
  security_margin_map_upper.triangularView<Eigen::Lower>().setZero();

// Apply the security margins
  geom_data_.setSecurityMargins(geom_model_, security_margin_map_upper, true, true);


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
  command_interfaces_config.names.push_back("robot/is_in_collision");


  return command_interfaces_config;
}

controller_interface::InterfaceConfiguration CollisionController::state_interface_configuration()
const
{
  controller_interface::InterfaceConfiguration state_interfaces_config;

  state_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  state_interfaces_config.names.reserve(params_.joints.size());
  for (const auto & dof_name : params_.joints) {
    state_interfaces_config.names.push_back(dof_name + "/position"); // to change into absolute position
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


  Eigen::VectorXd q = Eigen::VectorXd::Zero(params_.joints.size() + 7);

  q[6] = 1.0;
  for (const auto & joint : params_.joints) {
    q.tail(params_.joints.size())[model_.getJointId(joint) -
      2] = state_interfaces_[joint_id_[joint]].get_optional().value();
  }


  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn CollisionController::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type CollisionController::update_reference_from_subscribers(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{

  return controller_interface::return_type::OK;
}

controller_interface::return_type CollisionController::update_and_write_commands(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // check for any parameter updates
  //update_parameters();
  auto start = std::chrono::high_resolution_clock::now();

// Build the configuration vector `q`
  Eigen::VectorXd q = Eigen::VectorXd::Zero(params_.joints.size() + 7);
  q[6] = 1.0;

  for (const auto & joint : params_.joints) {
    size_t idx = model_.getJointId(joint) - 2;
    q.tail(params_.joints.size())[idx] = state_interfaces_[joint_id_[joint]].get_optional().value();
  }

  bool current_collision = false;

// Fast path: reuse last known collision pair
  if (collision_prev) {
    const auto & cr = geom_data_.collisionResults[collision_pair];
    if (cr.isCollision()) {
      if (!command_interfaces_[0].set_value(1.0)) {
        RCLCPP_ERROR(
          get_node()->get_logger(),
          "Failed to set command for robot_collision");
      }

      pinocchio::updateGeometryPlacements(model_, data_, geom_model_, geom_data_, q);
      pinocchio::computeAllTerms(model_, data_, q, Eigen::VectorXd::Zero(model_.nv));
      publish_collision_meshes();

      auto end = std::chrono::high_resolution_clock::now();
      auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
      return controller_interface::return_type::OK;

    }
  }

// Full collision check if no previous collision or collision cleared
  current_collision = pinocchio::computeCollisions(model_, data_, geom_model_, geom_data_, q, true);

  if (current_collision) {
    if (!command_interfaces_[0].set_value(1.0)) {
      RCLCPP_ERROR(
        get_node()->get_logger(),
        "Failed to set command for robot_collision");
    }


    pinocchio::updateGeometryPlacements(model_, data_, geom_model_, geom_data_, q);
    pinocchio::computeAllTerms(model_, data_, q, Eigen::VectorXd::Zero(model_.nv));
    publish_collision_meshes();


    for (size_t k = 0; k < geom_model_.collisionPairs.size(); ++k) {
      const auto & cr = geom_data_.collisionResults[k];
      if (cr.isCollision()) {
        collision_pair = k;

        const auto & cp = geom_model_.collisionPairs[k];
        const auto & obj1 = geom_model_.geometryObjects[cp.first];
        const auto & obj2 = geom_model_.geometryObjects[cp.second];

        RCLCPP_WARN_THROTTLE(
          get_node()->get_logger(),
          *get_node()->get_clock(),
          1000,
          "Collision detected between %s and %s",
          obj1.name.c_str(),
          obj2.name.c_str());

        return controller_interface::return_type::OK;
      }
    }
  } else {
    if (collision_prev) {
      RCLCPP_WARN(get_node()->get_logger(), "Collision has been resolved");
    }
    if (!command_interfaces_[0].set_value(0.0)) {
      {
        RCLCPP_ERROR(
          get_node()->get_logger(),
          "Failed to set command for robot_collision");
      }
    }
  }

  pinocchio::updateGeometryPlacements(model_, data_, geom_model_, geom_data_, q);
  pinocchio::computeAllTerms(model_, data_, q, Eigen::VectorXd::Zero(model_.nv));
  publish_collision_meshes();


  collision_prev = current_collision;

  auto end = std::chrono::high_resolution_clock::now();
  auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);


  return controller_interface::return_type::OK;
}

void CollisionController::removeCollisionObjectsForLinks(
  const std::vector<std::string> & link_names)
{
  // Create a list to store the names of the geometries to remove
  std::vector<std::string> to_remove;

  // Iterate through the geometry objects in the model
  for (const auto & obj : geom_model_.geometryObjects) {
    // Get the name of the parent joint for the current object
    std::string parent_link_name = model_.names[obj.parentJoint];

    // Check if the parent link is in the list of links to remove
    for (const auto & link_name : link_names) {
      if (obj.name.rfind(link_name, 0) == 0) {
        to_remove.push_back(obj.name);
      }
    }
  }

  // Remove the selected geometries
  for (const auto & name : to_remove) {
    geom_model_.removeGeometryObject(name);
  }

  // Clear any stale collision pairs and rebuild them
  geom_model_.collisionPairs.clear();
  geom_model_.addAllCollisionPairs();
}

void CollisionController::removeCollisionsAndAddSphere(
  const std::vector<std::string> & to_remove_names, double radius,
  std::string name_collision)
{
  // Step 1: Compute the bounding box that encompasses all the objects in `to_remove_names`
  Eigen::Vector3d min_corner = Eigen::Vector3d::Constant(std::numeric_limits<double>::max());
  Eigen::Vector3d max_corner = Eigen::Vector3d::Constant(std::numeric_limits<double>::lowest());


  // Step 1: Prepare for the new collision sphere (initializing it as null)
  std::shared_ptr<hpp::fcl::CollisionGeometry> new_sphere;

  // Step 2: Loop over the list of geometry objects and remove those with matching names
  std::vector<pinocchio::GeometryObject> collisionObjects;
  std::vector<std::string> to_remove;
  std::string base_link;
  std::vector<Eigen::Vector3d> positions;

  for (const auto & obj : geom_model_.geometryObjects) {
    for (const auto & name : to_remove_names) {
      if (obj.name.rfind(name, 0) == 0) {        // Matching name (starts with 'name')
        collisionObjects.push_back(obj);
        to_remove.push_back(obj.name);
        // Store the position of the object to be removed
        positions.push_back(obj.placement.translation());
        const auto & global_position = data_.oMi[obj.parentJoint].act(obj.placement).translation();
        min_corner = min_corner.cwiseMin(global_position);
        max_corner = max_corner.cwiseMax(global_position);

      }
    }
  }
  pinocchio::JointIndex base_joint_id;

  if (!collisionObjects.empty()) {
    const pinocchio::GeometryObject * last_removed_obj = nullptr;
    auto joint_id = collisionObjects[0].parentJoint;
    for (const auto & frame : model_.frames) {
      if (frame.parent == joint_id && frame.type == pinocchio::FrameType::BODY) {
        base_link = frame.name;
        break;
      }
    }

    for (auto obj : collisionObjects) {
      // Transform the position to the base link frame using the base link's placement
      const auto & base_link_transform = data_.oMi[model_.getJointId(base_link)];
      const auto & global_position =
        base_link_transform.act(data_.oMi[obj.parentJoint].act(obj.placement)).translation();
    }

    // Find the base link object
    for (const auto & obj : geom_model_.geometryObjects) {
      if (obj.name.rfind(base_link, 0) == 0) {
        last_removed_obj = &obj;
        break;
      }
    }

    // Step 3: Compute the average position to use as position of the collision
    Eigen::Vector3d average_position = Eigen::Vector3d::Zero();
    for (const auto & pos : positions) {
      average_position += pos;
    }
    if (!positions.empty()) {
      average_position /= static_cast<double>(positions.size());
    }

    // Step 3: Use the average position for the new sphere
    Eigen::Vector3d new_position = average_position;

    base_joint_id = last_removed_obj->parentJoint;
    // The new sphere will be positioned based on the average position computed
    Eigen::Quaterniond new_orientation = Eigen::Quaterniond::Identity();      // No rotation (default)

    // Step 4: Create the new sphere geometry with the provided radius
    new_sphere = std::make_shared<hpp::fcl::Sphere>(radius);

    // Step 2: Compute the dimensions of the parallelogram mesh
    Eigen::Vector3d dimensions = max_corner - min_corner;

    // Step 3: Compute the center of the parallelogram
    Eigen::Vector3d center = (min_corner + max_corner) / 2.0;

    // Step 4: Create the parallelogram mesh
    std::shared_ptr<hpp::fcl::CollisionGeometry> parallelogram_mesh =
      std::make_shared<hpp::fcl::Box>(
      dimensions.x() + 0.05, dimensions.y() + 0.05,
      dimensions.z() + 0.05);

    hpp::fcl::Transform3f tf;
    tf.setIdentity();      // Initialize the transformation as identity
    RCLCPP_INFO(
      get_node()->get_logger(), "New sphere position: %f %f %f",
      new_position.x(), new_position.y(), new_position.z());
    tf.translation() = new_position;   // Set the translation (position)
    // tf.translation()[0] -= 0.2;
    tf.rotation() = new_orientation;   // Set the rotation (orientation)

    // Step 6: Create the collision object for the new sphere
    hpp::fcl::CollisionObject sphere_coll_obj(new_sphere, tf);

    // Optional: Compute the AABB for the new sphere (for collision detection optimization)
    sphere_coll_obj.computeAABB();
    hpp::fcl::AABB sphere_aabb = sphere_coll_obj.getAABB();
    std::cout << "New Sphere AABB: min: " << sphere_aabb.min_.transpose() << ", max: " <<
      sphere_aabb.max_.transpose() << std::endl;

    // Step 7: Add the new sphere to the model
    pinocchio::GeometryObject sphere_geom_obj(
      name_collision,                // Name of the new object
      base_joint_id,                        // Attach to a specific joint ID
      parallelogram_mesh,                           // The sphere geometry
      pinocchio::SE3(tf.rotation(), tf.translation())        // The transformation (position and orientation)
    );

    // Add the new geometry object to the model
    geom_model_.addGeometryObject(sphere_geom_obj);

    // Remove the identified objects from the model
    for (const auto & name : to_remove) {
      geom_model_.removeGeometryObject(name);
    }

    // Clear any stale collision pairs and rebuild the collision pairs
    geom_model_.collisionPairs.clear();
    geom_model_.addAllCollisionPairs();
  }

}

void CollisionController::removeCollisionBetweenLinks(
  const std::string & link1,
  const std::string & link2)
{
  // Iterate through the collision pairs
  for (auto it = geom_model_.collisionPairs.begin(); it != geom_model_.collisionPairs.end();
    ++it)
  {
    const auto & pair = *it;

    // Retrieve geometry names based on the IDs or references (assuming IDs are used)
    std::string name1 = geom_model_.geometryObjects[pair.first].name;     // Add appropriate method to get name by ID
    std::string name2 = geom_model_.geometryObjects[pair.second].name;   // Add appropriate method to get name by ID

    // Check if the names match the given link names
    if ((name1 == link1 && name2 == link2) || (name1 == link2 && name2 == link1)) {
      std::cout << "Removing collision pair: " << name1 << " <-> " << name2 << std::endl;
      geom_model_.collisionPairs.erase(it);
      return;        // We can return early since we've removed the pair
    }
  }


  // If we reach here, the collision pair wasn't found
  std::cout << "Collision pair not found: " << link1 << " <-> " << link2 << std::endl;
}

void CollisionController::publish_collision_meshes()
{
  visualization_msgs::msg::Marker marker;
  int id = 0;

  for (const auto & obj : geom_model_.geometryObjects) {

    // Skip the object if it does not belong to a pair collision

    if (auto sphere = std::dynamic_pointer_cast<hpp::fcl::Sphere>(obj.geometry)) {
      marker.type = visualization_msgs::msg::Marker::SPHERE;
      marker.scale.x = sphere->radius * 2.0;
      marker.scale.y = sphere->radius * 2.0;
      marker.scale.z = sphere->radius * 2.0;
    } else if (auto box = std::dynamic_pointer_cast<hpp::fcl::Box>(obj.geometry)) {
      marker.type = visualization_msgs::msg::Marker::CUBE;
      marker.scale.x = box->halfSide[0] * 2.0;
      marker.scale.y = box->halfSide[1] * 2.0;
      marker.scale.z = box->halfSide[2] * 2.0;
    } else if (auto cylinder = std::dynamic_pointer_cast<hpp::fcl::Cylinder>(obj.geometry)) {
      marker.type = visualization_msgs::msg::Marker::CYLINDER;
      marker.scale.x = cylinder->radius * 2.0;
      marker.scale.y = cylinder->radius * 2.0;
      marker.scale.z = cylinder->halfLength * 2.0;
    } else {
      RCLCPP_WARN(
        get_node()->get_logger(), "Unknown geometry type for object: %s",
        obj.name.c_str());
      continue;
    }
    const auto & aabb = obj.geometry->aabb_local;
    const double dx = aabb.max_.x() - aabb.min_.x();
    const double dy = aabb.max_.y() - aabb.min_.y();
    const double dz = aabb.max_.z() - aabb.min_.z();

// Diameter of a bounding sphere
    const double diameter = std::sqrt(dx * dx + dy * dy + dz * dz);

    marker.header.frame_id = "base_footprint";  // Update to your robot's base frame
    marker.header.stamp = get_node()->now();
    marker.ns = "collision_mesh";
    marker.id = id++;
    marker.action = visualization_msgs::msg::Marker::ADD;

    // retrieve oMg vector from geom_data, to take the current position of the object
    const auto & oMg = data_.oMi[obj.parentJoint];
    // retrieve the placement of the object
    const auto & placement = oMg.act(obj.placement);


    //Stream the name of the object
    RCLCPP_INFO(
      get_node()->get_logger(), "Object parent joint: %s",
      model_.names[obj.parentJoint].c_str()
    );
    RCLCPP_INFO(
      get_node()->get_logger(), "Object name: %s", obj.name.c_str());

    // PRint placement of object
    RCLCPP_INFO(
      get_node()->get_logger(), "Object placement (oMg): %f, %f, %f",
      placement.translation()[0],
      placement.translation()[1],
      placement.translation()[2]);


    marker.pose.position.x = placement.translation()[0];
    marker.pose.position.y = placement.translation()[1];
    marker.pose.position.z = placement.translation()[2];

    Eigen::Quaterniond q(placement.rotation());
    marker.pose.orientation.x = q.x();
    marker.pose.orientation.y = q.y();
    marker.pose.orientation.z = q.z();
    marker.pose.orientation.w = q.w();

    // setting radius for the marker
    //marker.scale.x = diameter;
    //marker.scale.y = diameter;
    //marker.scale.z = diameter;

    marker.color.r = 1.0f;
    marker.color.g = 0.0f;
    marker.color.b = 0.0f;
    marker.color.a = 0.6f;

    marker.lifetime = rclcpp::Duration(0, 0);

    marker_pub_->publish(marker);

  }
}


}   // namespace collision_controller

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  collision_controller::CollisionController, controller_interface::ChainableControllerInterface)
