// # SPDX-FileCopyrightText: Alliander N. V.
//
// # SPDX-License-Identifier: Apache-2.0

#include <deque>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.hpp>
#include <moveit/robot_model/joint_model_group.hpp>
#include <moveit_msgs/srv/servo_command_type.hpp>
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
typedef moveit_msgs::srv::ServoCommandType ServoCommandType;
typedef std_srvs::srv::Trigger Trigger;
typedef geometry_msgs::msg::PoseStamped PoseStamped;
typedef rcdt_utilities_msgs::srv::ExpressPoseInOtherFrame
    ExpressPoseInOtherFrame;

struct Action {
  std::string name;
  std::string argument = "";
  float value = 0.0;
};

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
  PoseStamped goal_pose;

  //   Definitions:
  std::map<std::string, int> shapes = {
      {"BOX", 1}, {"SPHERE", 2}, {"CYLINDER", 3}, {"CONE", 4}};
  std::set<std::string> pilz_types = {"PTP", "LIN", "CIRC"};
  std::map<std::string, int> servo_command_types = {
      {"JOINT_JOG", 0}, {"TWIST", 1}, {"POSE", 2}};

  //   Clients:
  rclcpp::Client<ServoCommandType>::SharedPtr switch_servo_type_client;
  rclcpp::Client<Trigger>::SharedPtr open_gripper_client;
  rclcpp::Client<Trigger>::SharedPtr close_gripper_client;
  rclcpp::Client<ExpressPoseInOtherFrame>::SharedPtr
      express_pose_in_other_frame_client;

  //   Services:
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

  //   Methods:
  void initialize_clients();
  void initialize_services();
  void switch_servo_command_type(std::string command_type);
  bool open_gripper();
  bool close_gripper();
  PoseStamped change_frame_to_world(PoseStamped pose);
  bool transform_goal_pose(std::string axis, float value);
  bool plan_and_execute(std::string planning_type = "");
  bool execute_action(Action action);
  bool execute_sequence(std::deque<Action> sequence);
};