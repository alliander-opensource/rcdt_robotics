#include "moveit_servo.hpp"
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.hpp>
#include <rcdt_utilities_msgs/srv/add_object.hpp>
#include <rcdt_utilities_msgs/srv/detail/move_hand_to_pose__struct.hpp>
#include <rcdt_utilities_msgs/srv/move_hand_to_pose.hpp>
#include <rcdt_utilities_msgs/srv/move_to_configuration.hpp>
#include <rclcpp/node.hpp>
#include <std_srvs/srv/trigger.hpp>

typedef rcdt_utilities_msgs::srv::AddObject AddObject;
typedef rcdt_utilities_msgs::srv::MoveToConfiguration MoveToConf;
typedef rcdt_utilities_msgs::srv::MoveHandToPose MoveHandToPose;
typedef std_srvs::srv::Trigger Trigger;

class MoveitClient {
public:
  MoveitClient(rclcpp::Node::SharedPtr node);

private:
  rclcpp::Node::SharedPtr node;
  std::string planning_group = "fr3_arm";
  moveit::planning_interface::MoveGroupInterface move_group;
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

  std::map<std::string, int> shapes = {
      {"BOX", 1}, {"SPHERE", 2}, {"CYLINDER", 3}, {"CONE", 4}};

  rclcpp::Service<AddObject>::SharedPtr add_object_service;
  void add_object(const std::shared_ptr<AddObject::Request> request,
                  std::shared_ptr<AddObject::Response> response);

  rclcpp::Service<Trigger>::SharedPtr clear_objects_service;
  void clear_objects(const std::shared_ptr<Trigger::Request> request,
                     std::shared_ptr<Trigger::Response> response);

  rclcpp::Service<MoveToConf>::SharedPtr move_to_configuration_service;
  void move_to_configuration(const std::shared_ptr<MoveToConf::Request> request,
                             std::shared_ptr<MoveToConf::Response> response);

  rclcpp::Service<MoveHandToPose>::SharedPtr move_hand_to_pose_service;
  void move_hand_to_pose(const std::shared_ptr<MoveHandToPose::Request> request,
                         std::shared_ptr<MoveHandToPose::Response> response);

  MoveitServo moveit_servo;
};