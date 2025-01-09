#include "moveit_cpp.hpp"
#include "moveit_servo.hpp"
#include "planning_scene.hpp"
#include <rcdt_utilities_msgs/srv/move_to_configuration.hpp>
#include <rclcpp/node.hpp>

typedef rcdt_utilities_msgs::srv::MoveToConfiguration MoveToConf;

class MoveitController : public rclcpp::Node {
public:
  MoveitController(rclcpp::NodeOptions options);
  PlanningScene planning_scene;
  MoveitCpp moveit_cpp;
  MoveitServo moveit_servo;

  void initialize();
  rclcpp::Service<MoveToConf>::SharedPtr service;
  void move_to_configuration(const std::shared_ptr<MoveToConf::Request> request,
                             std::shared_ptr<MoveToConf::Response> response);
};
