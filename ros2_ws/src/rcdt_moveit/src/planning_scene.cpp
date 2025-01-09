#include "planning_scene.hpp"
#include <memory>
#include <moveit/planning_scene_monitor/planning_scene_monitor.hpp>

void PlanningScene::initialize(std::shared_ptr<rclcpp::Node> node) {
  robot_model_loader::RobotModelLoader robot_model_loader(node);
  planning_scene_monitor =
      std::make_shared<planning_scene_monitor::PlanningSceneMonitor>(
          node, robot_model_loader.getRobotDescription());

  planning_scene_monitor->startStateMonitor();
  planning_scene_monitor->startSceneMonitor();
  planning_scene_monitor->providePlanningSceneService();
};

planning_scene_monitor::PlanningSceneMonitorPtr
PlanningScene::get_planning_scene() {
  return planning_scene_monitor;
};