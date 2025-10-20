// # SPDX-FileCopyrightText: Alliander N. V.
//
// # SPDX-License-Identifier: Apache-2.0

#include "moveit_manager.hpp"
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/node_options.hpp>
#include <rclcpp/rclcpp.hpp>
#include <vector>

using std::placeholders::_1;
using std::placeholders::_2;

MoveitManager::MoveitManager(rclcpp::Node::SharedPtr node_,
                             std::string group_arm, std::string group_hand)
    : node(node_),
      move_group(
          node,
          moveit::planning_interface::MoveGroupInterface::Options(
              group_arm,
              moveit::planning_interface::MoveGroupInterface::ROBOT_DESCRIPTION,
              node->get_namespace())),
      rviz_visual_tools(base_frame, marker_topic, node),
      moveit_visual_tools(node, base_frame, marker_topic) {

  joint_model_group_arm =
      move_group.getRobotModel()->getJointModelGroup(group_arm);
  joint_model_group_hand =
      move_group.getRobotModel()->getJointModelGroup(group_hand);

  moveit_visual_tools.loadMarkerPub(false);
  moveit_visual_tools.loadRobotStatePub("display_robot_state");

  initialize_clients();
  initialize_services();
};

void MoveitManager::initialize_clients() {
  client_node = std::make_shared<rclcpp::Node>("moveit_manager_client");
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

  define_goal_pose_service = node->create_service<DefineGoalPose>(
      "~/define_goal_pose",
      std::bind(&MoveitManager::define_goal_pose, this, _1, _2));

  transform_goal_pose_service = node->create_service<TransformGoalPose>(
      "~/transform_goal_pose",
      std::bind(&MoveitManager::transform_goal_pose, this, _1, _2));

  move_to_configuration_service = node->create_service<MoveToConf>(
      "~/move_to_configuration",
      std::bind(&MoveitManager::move_to_configuration, this, _1, _2));

  move_hand_to_pose_service = node->create_service<MoveHandToPose>(
      "~/move_hand_to_pose",
      std::bind(&MoveitManager::move_hand_to_pose, this, _1, _2));

  add_marker_service = node->create_service<AddMarker>(
      "~/add_marker", std::bind(&MoveitManager::add_marker, this, _1, _2));

  visualize_gripper_pose_service = node->create_service<Trigger>(
      "~/visualize_gripper_pose",
      std::bind(&MoveitManager::visualize_gripper_pose, this, _1, _2));

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

  std::vector<moveit_msgs::msg::CollisionObject> collision_objects;
  solid_primitive.dimensions = {request->d1, request->d2, request->d3};
  collision_object.primitives.push_back(solid_primitive);
  collision_object.primitive_poses.push_back(request->pose.pose);
  collision_object.operation = collision_object.ADD;
  collision_object.id = "object";
  collision_objects.push_back(collision_object);
  planning_scene_interface.addCollisionObjects(collision_objects);
  response->success = true;
};

void MoveitManager::clear_objects(
    const std::shared_ptr<Trigger::Request> request,
    std::shared_ptr<Trigger::Response> response) {
  planning_scene_interface.removeCollisionObjects({"object"});
  response->success = true;
};

void MoveitManager::define_goal_pose(
    const std::shared_ptr<DefineGoalPose::Request> request,
    std::shared_ptr<DefineGoalPose::Response> response) {
  auto pose = change_frame_to_base(request->pose);
  goal_pose = pose;
  response->success = true;
};

void MoveitManager::transform_goal_pose(
    const std::shared_ptr<TransformGoalPose::Request> request,
    std::shared_ptr<TransformGoalPose::Response> response) {
  if (request->axis == "x") {
    goal_pose.pose.position.x += request->value;
  } else if (request->axis == "y") {
    goal_pose.pose.position.y += request->value;
  } else if (request->axis == "z") {
    goal_pose.pose.position.z += request->value;
  } else {
    RCLCPP_ERROR(node->get_logger(), "Axis must be one of 'x', 'y', 'z'.");
    response->success = false;
    return;
  }
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
  move_group.setPoseTarget(goal_pose);
  response->success = plan_and_execute(request->planning_type);
};

bool MoveitManager::plan_and_execute(std::string planning_type) {
  if (pilz_types.count(planning_type)) {
    move_group.setPlanningPipelineId("pilz_industrial_motion_planner");
    move_group.setPlannerId(planning_type);
  } else {
    move_group.setPlanningPipelineId("ompl");
  }
  moveit::planning_interface::MoveGroupInterface::Plan plan;
  auto error_code = move_group.plan(plan);
  if (error_code != moveit::core::MoveItErrorCode::SUCCESS) {
    RCLCPP_ERROR(node->get_logger(), "Failed to generate plan.");
    return false;
  }

  moveit_visual_tools.deleteAllMarkers("Path");
  moveit_visual_tools.deleteAllMarkers("Sphere");
  moveit_visual_tools.publishTrajectoryLine(plan.trajectory,
                                            joint_model_group_arm);
  moveit_visual_tools.trigger();

  error_code = move_group.execute(plan);
  if (error_code != moveit::core::MoveItErrorCode::SUCCESS) {
    RCLCPP_ERROR(node->get_logger(), "Failed to execute plan.");
    return false;
  }
  return true;
};

PoseStamped MoveitManager::change_frame_to_base(PoseStamped pose) {
  auto request = std::make_shared<ExpressPoseInOtherFrame::Request>();
  request->pose = pose;
  request->target_frame = base_frame;
  auto future = express_pose_in_other_frame_client->async_send_request(request);
  rclcpp::spin_until_future_complete(client_node, future);
  auto response = future.get();
  return response->pose;
};

void MoveitManager::add_marker(
    const std::shared_ptr<AddMarker::Request> request,
    std::shared_ptr<AddMarker::Response> response) {
  auto pose = change_frame_to_base(request->marker_pose);
  moveit_visual_tools.publishAxis(pose.pose);
  moveit_visual_tools.trigger();
  response->success = true;
};

void MoveitManager::visualize_gripper_pose(
    const std::shared_ptr<Trigger::Request> request,
    std::shared_ptr<Trigger::Response> response) {

  PoseStamped pose_hand;
  pose_hand.header.frame_id =
      joint_model_group_hand->getEndEffectorParentGroup().second;
  auto pose_map = change_frame_to_base(pose_hand);

  std::vector<double> positions = {0.04};
  moveit_visual_tools.publishEEMarkers(pose_map.pose, joint_model_group_hand,
                                       positions, rviz_visual_tools::ORANGE);
  response->success = true;
}

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
  auto moveit_manager = MoveitManager(node, "fr3_arm", "franka_hand");
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
}