// # SPDX-FileCopyrightText: Alliander N. V.
//
// # SPDX-License-Identifier: Apache-2.0

#include "moveit_manager.hpp"
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/node_options.hpp>
#include <rclcpp/rclcpp.hpp>

using std::placeholders::_1;
using std::placeholders::_2;

MoveitManager::MoveitManager(rclcpp::Node::SharedPtr node_)
    : node(node_), move_group(node, planning_group),
      moveit_visual_tools(node, "fr3_link0", "/rviz_markers") {

  move_group.setEndEffectorLink("fr3_hand");
  joint_model_group = move_group.getRobotModel()->getJointModelGroup("fr3_arm");

  initialize_clients();
  initialize_services();

  switch_servo_command_type("TWIST");
};

void MoveitManager::initialize_clients() {
  client_node = std::make_shared<rclcpp::Node>("moveit_manager_client");
  switch_servo_type_client = client_node->create_client<ServoCommandType>(
      "/servo_node/switch_command_type");
  open_gripper_client = client_node->create_client<Trigger>("/open_gripper");
  close_gripper_client = client_node->create_client<Trigger>("/close_gripper");
  express_pose_in_other_frame_client =
      client_node->create_client<ExpressPoseInOtherFrame>(
          "/express_pose_in_other_frame");
};

void MoveitManager::initialize_services() {
  add_object_service = node->create_service<AddObject>(
      "~/add_object", std::bind(&MoveitManager::add_object, this, _1, _2));

  clear_objects_service = node->create_service<Trigger>(
      "~/clear_objects",
      std::bind(&MoveitManager::clear_objects, this, _1, _2));

  move_to_configuration_service = node->create_service<MoveToConf>(
      "~/move_to_configuration",
      std::bind(&MoveitManager::move_to_configuration, this, _1, _2));

  move_hand_to_pose_service = node->create_service<MoveHandToPose>(
      "~/move_hand_to_pose",
      std::bind(&MoveitManager::move_hand_to_pose, this, _1, _2));

  pick_at_pose_service = node->create_service<MoveHandToPose>(
      "~/pick_at_pose", std::bind(&MoveitManager::pick_at_pose, this, _1, _2));

  drop_service = node->create_service<Trigger>(
      "~/drop", std::bind(&MoveitManager::drop, this, _1, _2));

  add_marker_service = node->create_service<AddMarker>(
      "~/add_marker", std::bind(&MoveitManager::add_marker, this, _1, _2));

  clear_markers_service = node->create_service<Trigger>(
      "~/clear_markers",
      std::bind(&MoveitManager::clear_markers, this, _1, _2));
};

void MoveitManager::add_object(
    const std::shared_ptr<AddObject::Request> request,
    std::shared_ptr<AddObject::Response> response) {
  moveit_msgs::msg::CollisionObject collision_object;
  collision_object.header = request->pose.header;

  shape_msgs::msg::SolidPrimitive solid_primitive;
  try {
    solid_primitive.type = shapes.at(request->shape);
  } catch (std::out_of_range) {
    RCLCPP_ERROR(node->get_logger(), "Shape %s is not a valid option.}. Exit.",
                 request->shape.c_str());
    return;
  }

  solid_primitive.dimensions = {request->d1, request->d2, request->d3};
  collision_object.primitives.push_back(solid_primitive);
  collision_object.primitive_poses.push_back(request->pose.pose);
  collision_object.operation = collision_object.ADD;
  collision_object.id = "object";
  planning_scene_interface.applyCollisionObject(collision_object);
  response->success = true;
};

void MoveitManager::clear_objects(
    const std::shared_ptr<Trigger::Request> request,
    std::shared_ptr<Trigger::Response> response) {
  planning_scene_interface.removeCollisionObjects({"object"});
  response->success = true;
};

void MoveitManager::move_to_configuration(
    const std::shared_ptr<MoveToConf::Request> request,
    std::shared_ptr<MoveToConf::Response> response) {
  move_group.setNamedTarget(request->configuration);
  response->success = plan_and_execute();
};

void MoveitManager::move_hand_to_pose(
    const std::shared_ptr<MoveHandToPose::Request> request,
    std::shared_ptr<MoveHandToPose::Response> response) {
  move_group.setPoseTarget(request->pose);
  response->success = plan_and_execute(request->planning_type);
};

void MoveitManager::pick_at_pose(
    const std::shared_ptr<MoveHandToPose::Request> request,
    std::shared_ptr<MoveHandToPose::Response> response) {
  auto pose = change_frame_to_world(request->pose);
  goal_pose = pose;

  std::deque<Action> sequence;
  sequence.push_back(Action{"open_gripper"});
  sequence.push_back(Action{"transform_goal_pose", "z", 0.15});
  sequence.push_back(Action{"plan_and_execute"});
  sequence.push_back(Action{"transform_goal_pose", "z", -0.07});
  sequence.push_back(Action{"plan_and_execute", "LIN"});
  sequence.push_back(Action{"close_gripper"});
  sequence.push_back(Action{"transform_goal_pose", "z", 0.07});
  sequence.push_back(Action{"plan_and_execute", "LIN"});
  sequence.push_back(Action{"set_named_target", "home"});
  sequence.push_back(Action{"plan_and_execute"});
  response->success = execute_sequence(sequence);
};

void MoveitManager::drop(const std::shared_ptr<Trigger::Request> request,
                         std::shared_ptr<Trigger::Response> response) {
  std::deque<Action> sequence;
  sequence.push_back(Action{"set_named_target", "drop"});
  sequence.push_back(Action{"plan_and_execute"});
  sequence.push_back(Action{"open_gripper"});
  sequence.push_back(Action{"close_gripper"});
  sequence.push_back(Action{"set_named_target", "home"});
  sequence.push_back(Action{"plan_and_execute"});
  response->success = execute_sequence(sequence);
};

bool MoveitManager::execute_sequence(std::deque<Action> sequence) {
  bool successful = true;
  while (successful && !sequence.empty()) {
    auto action = sequence.front();
    successful = execute_action(action);
    sequence.pop_front();
  }
  if (!successful) {
    RCLCPP_ERROR(node->get_logger(), "Failed to execute sequence.");
  }
  return successful;
};

bool MoveitManager::execute_action(Action action) {
  if (action.name == "open_gripper") {
    return open_gripper();
  }
  if (action.name == "close_gripper") {
    return close_gripper();
  }
  if (action.name == "set_named_target") {
    return move_group.setNamedTarget(action.argument);
  }
  if (action.name == "transform_goal_pose") {
    return transform_goal_pose(action.argument, action.value);
  }
  if (action.name == "plan_and_execute") {
    return plan_and_execute(action.argument);
  }
  RCLCPP_ERROR(node->get_logger(), "Action '%s' unkown. Exit.",
               action.name.c_str());
  return false;
}

bool MoveitManager::transform_goal_pose(std::string axis, float value) {
  if (axis == "x") {
    goal_pose.pose.position.x += value;
  } else if (axis == "y") {
    goal_pose.pose.position.y += value;
  } else if (axis == "z") {
    goal_pose.pose.position.z += value;
  } else {
    RCLCPP_ERROR(node->get_logger(),
                 "Can't transform pose: axis '%s' is unkown.", axis.c_str());
    return false;
  }
  move_group.setPoseTarget(goal_pose);
  return true;
};

bool MoveitManager::plan_and_execute(std::string planning_type) {
  if (pilz_types.count(planning_type)) {
    move_group.setPlanningPipelineId("pilz_industrial_motion_planner");
    move_group.setPlannerId(planning_type);
  } else {
    move_group.setPlanningPipelineId("ompl");
  }
  moveit::planning_interface::MoveGroupInterface::Plan plan;
  move_group.plan(plan);
  moveit_visual_tools.deleteAllMarkers("Path");
  moveit_visual_tools.deleteAllMarkers("Sphere");
  moveit_visual_tools.publishTrajectoryLine(plan.trajectory, joint_model_group);
  moveit_visual_tools.trigger();
  auto error_code = move_group.execute(plan);
  return (error_code == moveit::core::MoveItErrorCode::SUCCESS);
};

void MoveitManager::switch_servo_command_type(std::string command_type) {
  auto request = std::make_shared<ServoCommandType::Request>();

  try {
    request->command_type = servo_command_types.at(command_type);
  } catch (std::out_of_range) {
    RCLCPP_ERROR(node->get_logger(),
                 "Command type %s is not a valid option.}. Exit.",
                 command_type.c_str());
    return;
  }
  while (!switch_servo_type_client->wait_for_service(std::chrono::seconds(1))) {
    RCLCPP_WARN(node->get_logger(),
                "Servo node service not available. Waiting...");
  }
  auto future = switch_servo_type_client->async_send_request(request);
  rclcpp::spin_until_future_complete(client_node, future);
  auto response = future.get();
}

bool MoveitManager::open_gripper() {
  auto request = std::make_shared<Trigger::Request>();
  RCLCPP_INFO(client_node->get_logger(), "Opening gripper...");
  auto future = open_gripper_client->async_send_request(request);
  rclcpp::spin_until_future_complete(client_node, future);
  auto response = future.get();
  if (response->success) {
    RCLCPP_INFO(client_node->get_logger(), "Gripper opened.");
  } else {
    RCLCPP_WARN(client_node->get_logger(), "Failed to open gripper.");
  }
  return response->success;
};

bool MoveitManager::close_gripper() {
  auto request = std::make_shared<Trigger::Request>();
  RCLCPP_INFO(client_node->get_logger(), "Closing gripper...");
  auto future = close_gripper_client->async_send_request(request);
  rclcpp::spin_until_future_complete(client_node, future);
  auto response = future.get();
  if (response->success) {
    RCLCPP_INFO(client_node->get_logger(), "Gripper closed.");
  } else {
    RCLCPP_WARN(client_node->get_logger(), "Failed to close gripper.");
  }
  return response->success;
};

PoseStamped MoveitManager::change_frame_to_world(PoseStamped pose) {
  auto request = std::make_shared<ExpressPoseInOtherFrame::Request>();
  request->pose = pose;
  request->target_frame = "world";
  auto future = express_pose_in_other_frame_client->async_send_request(request);
  rclcpp::spin_until_future_complete(client_node, future);
  auto response = future.get();
  return response->pose;
};

void MoveitManager::add_marker(
    const std::shared_ptr<AddMarker::Request> request,
    std::shared_ptr<AddMarker::Response> response) {
  moveit_visual_tools.publishAxis(request->marker_pose.pose);
  moveit_visual_tools.trigger();
  response->success = true;
};

void MoveitManager::clear_markers(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
  moveit_visual_tools.deleteAllMarkers("Axis");
  moveit_visual_tools.trigger();
  response->success = true;
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);
  auto node = std::make_shared<rclcpp::Node>("moveit_manager", node_options);
  auto moveit_manager = MoveitManager(node);
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
}