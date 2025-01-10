#include <moveit/planning_scene_monitor/planning_scene_monitor.hpp>
#include <rcdt_utilities_msgs/srv/add_object.hpp>
#include <rclcpp/node.hpp>
#include <std_srvs/srv/trigger.hpp>

typedef rcdt_utilities_msgs::srv::AddObject AddObject;
typedef std_srvs::srv::Trigger Trigger;

class PlanningScene {
public:
  void initialize(
      std::shared_ptr<rclcpp::Node> node,
      planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor);

private:
  std::shared_ptr<rclcpp::Node> node;
  planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor;

  std::map<std::string, int> shapes = {
      {"BOX", 1}, {"SPHERE", 2}, {"CYLINDER", 3}, {"CONE", 4}};
  rclcpp::Service<AddObject>::SharedPtr add_object_service;
  void add_object(const std::shared_ptr<AddObject::Request> request,
                  std::shared_ptr<AddObject::Response> response);

  rclcpp::Service<Trigger>::SharedPtr clear_objects_service;
  void clear_objects(const std::shared_ptr<Trigger::Request> request,
                     std::shared_ptr<Trigger::Response> response);
};