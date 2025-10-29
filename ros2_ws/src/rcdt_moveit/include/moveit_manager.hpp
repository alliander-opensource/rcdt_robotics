// # SPDX-FileCopyrightText: Alliander N. V.
//
// # SPDX-License-Identifier: Apache-2.0

#include <moveit_visual_tools/moveit_visual_tools.h>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.hpp>
#include <moveit/robot_model/joint_model_group.hpp>
#include <moveit_msgs/srv/servo_command_type.hpp>
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

/**
 * Class to interact with the Moveit framework.
 */
class MoveitManager {
 public:
  /**
   * @brief constructor for the MoveitManager class.
   * @param node The ROS2 node to attach to.
   */
  MoveitManager(rclcpp::Node::SharedPtr node);

 private:
  rclcpp::Node::SharedPtr node; /**< ROS2 node with MoveIt Services */
  rclcpp::Node::SharedPtr
      client_node; /**< ROS2 node with ExpressPoseInOtherFrame Client */
  moveit::planning_interface::MoveGroupInterface
      move_group; /**< Interface for setting joint/pose goals */
  moveit::planning_interface::PlanningSceneInterface
      planning_scene_interface; /**< Interface for adding collision objects */
  const moveit::core::JointModelGroup*
      joint_model_group; /**< Joint models for planning trajectories */
  moveit_visual_tools::MoveItVisualTools
      moveit_visual_tools; /**< Visualization tools */
  PoseStamped goal_pose;   /**< Goal pose to move end-effector to */

  //   Definitions:
  std::map<std::string, int> shapes = {
      {"BOX", 1},
      {"SPHERE", 2},
      {"CYLINDER", 3},
      {"CONE", 4}}; /**< Basic shape types for MoveIt */
  std::set<std::string> pilz_types = {
      "PTP", "LIN",
      "CIRC"}; /**< Motion planning types (point-to-point, linear, circular) */

  //   Clients:
  rclcpp::Client<ExpressPoseInOtherFrame>::SharedPtr
      express_pose_in_other_frame_client; /**< Client to transform a Pose in
                                             another coordinate frame */

  //   Services:
  rclcpp::Service<AddObject>::SharedPtr
      add_object_service; /**< Service to add collision object */

  /**
   * @brief Callback to add a collision object.
   * @param request The AddObject request indicating object pose and shape.
   * @param response Response indicating whether the service call succeeded.
   */
  void add_object(const std::shared_ptr<AddObject::Request> request,
                  std::shared_ptr<AddObject::Response> response);

  rclcpp::Service<Trigger>::SharedPtr
      clear_objects_service; /**< Service to remove all collision objects */

  /**
   * @brief Callback to clear collision objects.
   * @param request The Trigger request.
   * @param response Response indicating whether the service call succeeded.
   */
  void clear_objects(const std::shared_ptr<Trigger::Request> request,
                     std::shared_ptr<Trigger::Response> response);

  rclcpp::Service<DefineGoalPose>::SharedPtr
      define_goal_pose_service; /**< Service to define goal
                                   pose for arm to move to */

  /**
   * @brief Callback to define goal pose for arm to move to.
   * @param request The DefineGoalPose request containing the goal pose.
   * @param response Response indicating whether the service call succeeded.
   */
  void define_goal_pose(const std::shared_ptr<DefineGoalPose::Request> request,
                        std::shared_ptr<DefineGoalPose::Response> response);

  rclcpp::Service<TransformGoalPose>::SharedPtr
      transform_goal_pose_service; /**< Service to offset the goal_pose by a
                                      specified translation */

  /**
   * @brief Callback to offset goal_pose by a specified translation around a
   * specified axis.
   * @param request The TransformGoalPose request containing relevant axis and
   * translation.
   * @param response Response indicating whether the service call succeeded.
   */
  void transform_goal_pose(
      const std::shared_ptr<TransformGoalPose::Request> request,
      std::shared_ptr<TransformGoalPose::Response> response);

  rclcpp::Service<MoveToConf>::SharedPtr
      move_to_configuration_service; /**< Service to move to a specified
                                        configuration */

  /**
   * @brief Callback to move arm to a specified configuration (e.g. home).
   * @param request The MoveToConf request containing the configuration.
   * @param response Response indicating whether the service call succeeded.
   */
  void move_to_configuration(const std::shared_ptr<MoveToConf::Request> request,
                             std::shared_ptr<MoveToConf::Response> response);

  rclcpp::Service<MoveHandToPose>::SharedPtr
      move_hand_to_pose_service; /**< Service to move hand to
                                    a specified goal Pose */

  /**
   * @brief Callback to move arm to a specified goal Pose.
   * @param request The MoveHandToPose request containing the goal Pose and Pilz
   * planning type.
   * @param response Response indicating whether the service call succeeded.
   */
  void move_hand_to_pose(const std::shared_ptr<MoveHandToPose::Request> request,
                         std::shared_ptr<MoveHandToPose::Response> response);

  rclcpp::Service<AddMarker>::SharedPtr
      add_marker_service; /**< Service to add a visual marker at a
                             specified Pose */

  /**
   * @brief Callback to add a visual marker at a specified location.
   * @param request The AddMarker request containing the Pose to add the visual
   * marker.
   * @param response Response indicating whether the service call succeeded.
   */
  void add_marker(const std::shared_ptr<AddMarker::Request> request,
                  std::shared_ptr<AddMarker::Response> response);

  rclcpp::Service<Trigger>::SharedPtr
      clear_markers_service; /**< Service to clear all visual markers. */

  /**
   * @brief Callback to clear all visual markers.
   * @param request The Trigger request to remove all visual markers.
   * @param response Response indicating whether the service call succeeded.
   */
  void clear_markers(const std::shared_ptr<Trigger::Request> request,
                     std::shared_ptr<Trigger::Response> response);

  //   Methods:
  /**
   * @brief Initializes the ExpressPoseInOtherFrame service client Node and
   * Client.
   */
  void initialize_clients();

  /**
   * @brief Initializes all MoveIt Services
   */
  void initialize_services();

  /**
   * @brief Method to change a specified Pose to a world-fixed frame.
   * @param pose PoseStamped object to transform into world-fixed-frame.
   * @return PoseStamped object transformed to world-fixed frame.
   */
  PoseStamped change_frame_to_world(PoseStamped pose);

  /**
   * @brief Method that engages the move_group to plan a trajectory to the
   * goal_pose.
   * @param planning_type String containing the Pilz planning type
   * (point-to-point, linear, circular).
   * @return boolean indicating whether the planning succeeded or not.
   */
  bool plan_and_execute(std::string planning_type = "");
};
