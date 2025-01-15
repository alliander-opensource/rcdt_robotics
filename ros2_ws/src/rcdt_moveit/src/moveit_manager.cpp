#include "moveit_manager.hpp"
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <rclcpp/node_options.hpp>
#include <rclcpp/rclcpp.hpp>

using std::placeholders::_1;
using std::placeholders::_2;

MoveitManager::MoveitManager(rclcpp::Node::SharedPtr node_)
    : node(node_), move_group(node, planning_group),
      moveit_visual_tools(node, "fr3_link0", "/rviz_markers") {
  move_group.setEndEffectorLink("fr3_hand");
  joint_model_group = move_group.getRobotModel()->getJointModelGroup("fr3_arm");
  moveit_servo.initialize(node);
  moveit_servo.activate();

  client_node = std::make_shared<rclcpp::Node>("moveit_manager_client");
  open_gripper_client = client_node->create_client<Trigger>("/open_gripper");
  close_gripper_client = client_node->create_client<Trigger>("/close_gripper");
  express_pose_in_other_frame_client =
      client_node->create_client<ExpressPoseInOtherFrame>(
          "/express_pose_in_other_frame");

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
  plan_and_execute();
  response->success = true;
};

void MoveitManager::move_hand_to_pose(
    const std::shared_ptr<MoveHandToPose::Request> request,
    std::shared_ptr<MoveHandToPose::Response> response) {
  move_group.setPoseTarget(request->pose);
  plan_and_execute(request->planning_type);
  response->success = true;
};

void MoveitManager::pick_at_pose(
    const std::shared_ptr<MoveHandToPose::Request> request,
    std::shared_ptr<MoveHandToPose::Response> response) {
  auto pose = change_frame_to_world(request->pose);
  auto z = &pose.pose.position.z;
  open_gripper();
  *z += 0.15;
  move_group.setPoseTarget(pose);
  plan_and_execute();
  *z -= 0.07;
  move_group.setPoseTarget(pose);
  plan_and_execute("LIN");
  close_gripper();
  *z += 0.07;
  move_group.setPoseTarget(pose);
  plan_and_execute("LIN");
  move_group.setNamedTarget("home");
  plan_and_execute();
  response->success = true;
};

void MoveitManager::drop(const std::shared_ptr<Trigger::Request> request,
                         std::shared_ptr<Trigger::Response> response) {
  move_group.setNamedTarget("drop");
  plan_and_execute();
  open_gripper();
  close_gripper();
  move_group.setNamedTarget("home");
  plan_and_execute();
  response->success = true;
};

void MoveitManager::plan_and_execute(std::string planning_type) {
  moveit_servo.deactivate();
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
  move_group.execute(plan);
  moveit_servo.activate();
};

void MoveitManager::open_gripper() {
  auto request = std::make_shared<Trigger::Request>();
  RCLCPP_INFO(client_node->get_logger(), "Opening gripper...");
  auto future = open_gripper_client->async_send_request(request);
  rclcpp::spin_until_future_complete(client_node, future);
  RCLCPP_INFO(client_node->get_logger(), "Gripper opened.");
};

void MoveitManager::close_gripper() {
  auto request = std::make_shared<Trigger::Request>();
  RCLCPP_INFO(client_node->get_logger(), "Closing gripper...");
  auto future = close_gripper_client->async_send_request(request);
  rclcpp::spin_until_future_complete(client_node, future);
  RCLCPP_INFO(client_node->get_logger(), "Gripper closed.");
};

geometry_msgs::msg::PoseStamped
MoveitManager::change_frame_to_world(geometry_msgs::msg::PoseStamped pose) {
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