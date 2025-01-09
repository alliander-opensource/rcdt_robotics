#include <moveit/planning_scene_monitor/planning_scene_monitor.hpp>
#include <rclcpp/node.hpp>

class PlanningScene {
public:
  void initialize(std::shared_ptr<rclcpp::Node> node);
  planning_scene_monitor::PlanningSceneMonitorPtr get_planning_scene();

private:
  planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor;
};