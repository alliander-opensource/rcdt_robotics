#include "moveit_servo.hpp"
#include <memory>
#include <moveit_servo/servo.hpp>
#include <sensor_msgs/msg/detail/joint_state__struct.hpp>

using std::placeholders::_1;

void MoveitServo::initialize(std::shared_ptr<rclcpp::Node> node_) {
  node = node_;

  // Get the servo parameters.
  auto param_namespace = "moveit_servo";
  auto servo_param_listener =
      std::make_shared<const servo::ParamListener>(node, param_namespace);
  params = servo_param_listener->get_params();

  // Create the joint_trajectory publisher.
  auto trajectory_outgoing_cmd_pub_ =
      node->create_publisher<trajectory_msgs::msg::JointTrajectory>(
          params.command_out_topic, rclcpp::SystemDefaultsQoS());

  //   Create the servo object.
  planning_scene_monitor = moveit_servo::createPlanningSceneMonitor(
      node_, servo_param_listener->get_params());
  ;
  planning_scene_monitor->startSceneMonitor("/monitored_planning_scene");
  servo = std::make_unique<moveit_servo::Servo>(node, servo_param_listener,
                                                planning_scene_monitor);

  // Set the command type for servo.
  servo->setCommandType(moveit_servo::CommandType::TWIST);
  command.velocities = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  command.frame_id = "fr3_link0";

  // Create publisher.
  pub = node->create_publisher<trajectory_msgs::msg::JointTrajectory>(
      params.command_out_topic, rclcpp::SystemDefaultsQoS());

  // Create subscribtion:
  sub = node->create_subscription<geometry_msgs::msg::TwistStamped>(
      "/servo_node/delta_twist_cmds", 10,
      std::bind(&MoveitServo::callback, this, _1));

  // Create:
  robot_state = planning_scene_monitor->getStateMonitor()->getCurrentState();
  joint_model_group = robot_state->getJointModelGroup(params.move_group_name);

  // Start:
  activate();
  thread = std::thread(&MoveitServo::servo_loop, this);
};

void MoveitServo::activate() {
  joint_cmd_rolling_window.clear();
  auto joint_state = servo->getCurrentRobotState(true);
  servo->resetSmoothing(joint_state);
  updateSlidingWindow(joint_state, joint_cmd_rolling_window,
                      params.max_expected_latency, node->now());
  update_robot_state();
  active = true;
}

void MoveitServo::deactivate() { active = false; }

void MoveitServo::callback(
    const geometry_msgs::msg::TwistStamped::SharedPtr msg) {
  command.velocities = {msg->twist.linear.x,  msg->twist.linear.y,
                        msg->twist.linear.z,  msg->twist.angular.x,
                        msg->twist.angular.y, msg->twist.angular.z};
}

void MoveitServo::update_robot_state() {
  if (!joint_cmd_rolling_window.empty()) {
    robot_state->setJointGroupPositions(
        joint_model_group, joint_cmd_rolling_window.back().positions);
    robot_state->setJointGroupVelocities(
        joint_model_group, joint_cmd_rolling_window.back().velocities);
  }
}

void MoveitServo::servo_loop() {
  rclcpp::WallRate rate(1.0 / params.publish_period);
  while (rclcpp::ok()) {
    auto status = servo->getStatus();

    if (status != moveit_servo::StatusCode::INVALID && active) {
      auto joint_state = servo->getNextJointState(robot_state, command);
      updateSlidingWindow(joint_state, joint_cmd_rolling_window,
                          params.max_expected_latency, node->now());
      if (const auto msg =
              composeTrajectoryMessage(params, joint_cmd_rolling_window)) {
        pub->publish(msg.value());
      }
      update_robot_state();
    }
    rate.sleep();
  }
}