#include "moveit_servo.hpp"
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.hpp>
#include <moveit/robot_model/joint_model_group.hpp>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <rcdt_utilities_msgs/srv/add_marker.hpp>
#include <rcdt_utilities_msgs/srv/add_object.hpp>
#include <rcdt_utilities_msgs/srv/detail/move_hand_to_pose__struct.hpp>
#include <rcdt_utilities_msgs/srv/express_pose_in_other_frame.hpp>
#include <rcdt_utilities_msgs/srv/move_hand_to_pose.hpp>
#include <rcdt_utilities_msgs/srv/move_to_configuration.hpp>
#include <rclcpp/node.hpp>
#include <std_srvs/srv/trigger.hpp>

typedef rcdt_utilities_msgs::srv::AddObject AddObject;
typedef rcdt_utilities_msgs::srv::AddMarker AddMarker;
typedef rcdt_utilities_msgs::srv::MoveToConfiguration MoveToConf;
typedef rcdt_utilities_msgs::srv::MoveHandToPose MoveHandToPose;
typedef std_srvs::srv::Trigger Trigger;
typedef rcdt_utilities_msgs::srv::ExpressPoseInOtherFrame
    ExpressPoseInOtherFrame;

class MoveitManager {
public:
  MoveitManager(rclcpp::Node::SharedPtr node);

private:
  rclcpp::Node::SharedPtr node;
  rclcpp::Node::SharedPtr client_node;
  std::string planning_group = "fr3_arm";
  moveit::planning_interface::MoveGroupInterface move_group;
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  const moveit::core::JointModelGroup *joint_model_group;
  moveit_visual_tools::MoveItVisualTools moveit_visual_tools;

  std::map<std::string, int> shapes = {
      {"BOX", 1}, {"SPHERE", 2}, {"CYLINDER", 3}, {"CONE", 4}};
  std::set<std::string> pilz_types = {"PTP", "LIN", "CIRC"};

  rclcpp::Client<Trigger>::SharedPtr open_gripper_client;
  rclcpp::Client<Trigger>::SharedPtr close_gripper_client;
  rclcpp::Client<ExpressPoseInOtherFrame>::SharedPtr
      express_pose_in_other_frame_client;

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

  rclcpp::Service<MoveHandToPose>::SharedPtr pick_at_pose_service;
  void pick_at_pose(const std::shared_ptr<MoveHandToPose::Request> request,
                    std::shared_ptr<MoveHandToPose::Response> response);

  rclcpp::Service<Trigger>::SharedPtr drop_service;
  void drop(const std::shared_ptr<Trigger::Request> request,
            std::shared_ptr<Trigger::Response> response);

  rclcpp::Service<AddMarker>::SharedPtr add_marker_service;
  void add_marker(const std::shared_ptr<AddMarker::Request> request,
                  std::shared_ptr<AddMarker::Response> response);

  rclcpp::Service<Trigger>::SharedPtr clear_markers_service;
  void clear_markers(const std::shared_ptr<Trigger::Request> request,
                     std::shared_ptr<Trigger::Response> response);

  void open_gripper();
  void close_gripper();
  geometry_msgs::msg::PoseStamped
      change_frame_to_world(geometry_msgs::msg::PoseStamped);
  void plan_and_execute(std::string planning_type = "");

  MoveitServo moveit_servo;
};