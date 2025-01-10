#include "planning_scene.hpp"
#include <geometry_msgs/msg/detail/pose_stamped__struct.hpp>
#include <memory>
#include <moveit/planning_interface/planning_interface.hpp>
#include <rclcpp/logging.hpp>
#include <stdexcept>

using std::placeholders::_1;
using std::placeholders::_2;

void PlanningScene::initialize(std::shared_ptr<rclcpp::Node> node_) {
  node = node_;
  robot_model_loader::RobotModelLoader robot_model_loader(node);
  planning_scene_monitor =
      std::make_shared<planning_scene_monitor::PlanningSceneMonitor>(
          node, robot_model_loader.getRobotDescription());

  planning_scene_monitor->startStateMonitor();
  planning_scene_monitor->startSceneMonitor();
  planning_scene_monitor->providePlanningSceneService();

  add_object_service = node->create_service<AddObject>(
      "~/add_object", std::bind(&PlanningScene::add_object, this, _1, _2));
  clear_objects_service = node->create_service<Trigger>(
      "~/clear_objects",
      std::bind(&PlanningScene::clear_objects, this, _1, _2));
};

planning_scene_monitor::PlanningSceneMonitorPtr
PlanningScene::get_planning_scene() {
  return planning_scene_monitor;
};

void PlanningScene::add_object(
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
  {
    auto planning_scene =
        planning_scene_monitor::LockedPlanningSceneRW(planning_scene_monitor);
    planning_scene->processCollisionObjectMsg(collision_object);
  }
  response->success = true;
};

void PlanningScene::clear_objects(
    const std::shared_ptr<Trigger::Request> request,
    std::shared_ptr<Trigger::Response> response) {
  {
    auto planning_scene =
        planning_scene_monitor::LockedPlanningSceneRW(planning_scene_monitor);
    planning_scene->removeAllCollisionObjects();
  }
  response->success = true;
};