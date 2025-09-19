// # SPDX-FileCopyrightText: Alliander N. V.
//
// # SPDX-License-Identifier: Apache-2.0

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.hpp>
#include <moveit/robot_model/joint_model_group.hpp>
#include <moveit_msgs/srv/servo_command_type.hpp>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <rcdt_messages/srv/add_marker.hpp>
#include <rcdt_messages/srv/add_object.hpp>
#include <rcdt_messages/srv/define_goal_pose.hpp>
#include <rcdt_messages/srv/express_pose_in_other_frame.hpp>
#include <rcdt_messages/srv/move_hand_to_pose.hpp>
#include <rcdt_messages/srv/move_to_configuration.hpp>
#include <rcdt_messages/srv/transform_goal_pose.hpp>
#include <rclcpp/node.hpp>
#include <std_srvs/srv/trigger.hpp>

typedef rcdt_messages::srv::AddObject AddObject;
typedef rcdt_messages::srv::AddMarker AddMarker;
typedef rcdt_messages::srv::DefineGoalPose DefineGoalPose;
typedef rcdt_messages::srv::TransformGoalPose TransformGoalPose;
typedef rcdt_messages::srv::MoveToConfiguration MoveToConf;
typedef rcdt_messages::srv::MoveHandToPose MoveHandToPose;
typedef moveit_msgs::srv::ServoCommandType ServoCommandType;
typedef std_srvs::srv::Trigger Trigger;
typedef geometry_msgs::msg::PoseStamped PoseStamped;
typedef rcdt_messages::srv::ExpressPoseInOtherFrame ExpressPoseInOtherFrame;

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
  moveit::planning_interface::MoveGroupInterface move_group;
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  const moveit::core::JointModelGroup *joint_model_group;
  moveit_visual_tools::MoveItVisualTools moveit_visual_tools;
  PoseStamped goal_pose;

  //   Definitions:
  std::map<std::string, int> shapes = {
      {"BOX", 1}, {"SPHERE", 2}, {"CYLINDER", 3}, {"CONE", 4}};
  std::set<std::string> pilz_types = {"PTP", "LIN", "CIRC"};

  //   Clients:
  rclcpp::Client<ExpressPoseInOtherFrame>::SharedPtr
      express_pose_in_other_frame_client;

  //   Services:
  rclcpp::Service<AddObject>::SharedPtr add_object_service;
  void add_object(const std::shared_ptr<AddObject::Request> request,
                  std::shared_ptr<AddObject::Response> response);

  rclcpp::Service<Trigger>::SharedPtr clear_objects_service;
  void clear_objects(const std::shared_ptr<Trigger::Request> request,
                     std::shared_ptr<Trigger::Response> response);

  rclcpp::Service<DefineGoalPose>::SharedPtr define_goal_pose_service;
  void define_goal_pose(const std::shared_ptr<DefineGoalPose::Request> request,
                        std::shared_ptr<DefineGoalPose::Response> response);

  rclcpp::Service<TransformGoalPose>::SharedPtr transform_goal_pose_service;
  void
  transform_goal_pose(const std::shared_ptr<TransformGoalPose::Request> request,
                      std::shared_ptr<TransformGoalPose::Response> response);

  rclcpp::Service<MoveToConf>::SharedPtr move_to_configuration_service;
  void move_to_configuration(const std::shared_ptr<MoveToConf::Request> request,
                             std::shared_ptr<MoveToConf::Response> response);

  rclcpp::Service<MoveHandToPose>::SharedPtr move_hand_to_pose_service;
  void move_hand_to_pose(const std::shared_ptr<MoveHandToPose::Request> request,
                         std::shared_ptr<MoveHandToPose::Response> response);

  rclcpp::Service<AddMarker>::SharedPtr add_marker_service;
  void add_marker(const std::shared_ptr<AddMarker::Request> request,
                  std::shared_ptr<AddMarker::Response> response);

  rclcpp::Service<Trigger>::SharedPtr clear_markers_service;
  void clear_markers(const std::shared_ptr<Trigger::Request> request,
                     std::shared_ptr<Trigger::Response> response);

  //   Methods:
  void initialize_clients();
  void initialize_services();
  PoseStamped change_frame_to_base(PoseStamped pose);
  bool plan_and_execute(std::string planning_type = "");
};