#include "moveit_client.hpp"
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <rclcpp/node_options.hpp>
#include <rclcpp/rclcpp.hpp>

using std::placeholders::_1;
using std::placeholders::_2;

MoveitClient::MoveitClient(rclcpp::Node::SharedPtr node_)
    : node(node_), move_group(node, planning_group),
      moveit_visual_tools(node, "fr3_link0", "/rviz_markers") {
  move_group.setEndEffectorLink("fr3_hand");
  joint_model_group = move_group.getRobotModel()->getJointModelGroup("fr3_arm");
  moveit_servo.initialize(node);
  moveit_servo.activate();

  add_object_service = node->create_service<AddObject>(
      "~/add_object", std::bind(&MoveitClient::add_object, this, _1, _2));

  clear_objects_service = node->create_service<Trigger>(
      "~/clear_objects", std::bind(&MoveitClient::clear_objects, this, _1, _2));

  move_to_configuration_service = node->create_service<MoveToConf>(
      "~/move_to_configuration",
      std::bind(&MoveitClient::move_to_configuration, this, _1, _2));

  move_hand_to_pose_service = node->create_service<MoveHandToPose>(
      "~/move_hand_to_pose",
      std::bind(&MoveitClient::move_hand_to_pose, this, _1, _2));
};

void MoveitClient::add_object(const std::shared_ptr<AddObject::Request> request,
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

void MoveitClient::clear_objects(
    const std::shared_ptr<Trigger::Request> request,
    std::shared_ptr<Trigger::Response> response) {
  planning_scene_interface.removeCollisionObjects({"object"});
  response->success = true;
};

void MoveitClient::move_to_configuration(
    const std::shared_ptr<MoveToConf::Request> request,
    std::shared_ptr<MoveToConf::Response> response) {
  move_group.setNamedTarget(request->configuration);
  plan_and_execute();
  response->success = true;
};

void MoveitClient::move_hand_to_pose(
    const std::shared_ptr<MoveHandToPose::Request> request,
    std::shared_ptr<MoveHandToPose::Response> response) {
  move_group.setPoseTarget(request->pose);
  plan_and_execute(request->planning_type);
  response->success = true;
};

void MoveitClient::plan_and_execute(std::string planning_type) {
  moveit_servo.deactivate();
  if (pilz_types.count(planning_type)) {
    move_group.setPlanningPipelineId("pilz_industrial_motion_planner");
    move_group.setPlannerId(planning_type);
  } else {
    move_group.setPlanningPipelineId("ompl");
  }
  moveit::planning_interface::MoveGroupInterface::Plan plan;
  move_group.plan(plan);
  moveit_visual_tools.deleteAllMarkers();
  moveit_visual_tools.publishTrajectoryLine(plan.trajectory, joint_model_group);
  moveit_visual_tools.trigger();
  move_group.execute(plan);
  moveit_servo.activate();
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);
  auto node = std::make_shared<rclcpp::Node>("moveit_client", node_options);
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  auto moveit_client = MoveitClient(node);
  executor.spin();
  rclcpp::shutdown();
}