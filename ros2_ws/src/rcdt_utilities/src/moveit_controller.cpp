#include <cstdlib>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <memory>
#include <moveit/kinematics_base/kinematics_base.hpp>
#include <moveit/utils/logger.hpp>
#include <moveit_servo/servo.hpp>
#include <moveit_servo/utils/common.hpp>
#include <moveit_servo/utils/datatypes.hpp>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/transform_listener.h>
#include <thread>

using namespace moveit_servo;
using std::placeholders::_1;

class DebugMethods {
public:
  void print_state_from_robot_state(moveit::core::RobotStatePtr state) {
    for (int i = 1; i < 8; i++) {
      std::string joint_name = "fr3_joint" + std::to_string(i);
      std::cout << " pos: " << *state->getJointPositions(joint_name)
                << " vel: " << *state->getJointVelocities(joint_name)
                << std::endl;
    }
  }

  void print_state_from_kinematic_state(KinematicState state) {
    for (int i = 0; i < 7; i++) {
      std::string joint_name = "fr3_joint" + std::to_string(i);
      std::cout << "name:" << state.joint_names[i]
                << " pos: " << state.positions[i]
                << " vel: " << state.velocities[i] << std::endl;
    }
  }
};

class MoveitController : public rclcpp::Node {
public:
  MoveitController() : Node("moveit_controller") {
    twist_subscription_ =
        this->create_subscription<geometry_msgs::msg::TwistStamped>(
            "/servo_node/delta_twist_cmds", 10,
            std::bind(&MoveitController::callback, this, _1));
  }

  void initialize_moveit() {
    // The servo object expects to get a ROS node.
    rclcpp::Node::SharedPtr ptr_ = this->shared_from_this();
    moveit::setNodeLoggerName(ptr_->get_name());

    // Get the servo parameters.
    const std::string param_namespace = "moveit_servo";
    const std::shared_ptr<const servo::ParamListener> servo_param_listener =
        std::make_shared<const servo::ParamListener>(ptr_, param_namespace);
    servo_params_ = servo_param_listener->get_params();

    // Create the joint_trajectory publisher.
    trajectory_outgoing_cmd_pub_ =
        this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
            servo_params_.command_out_topic, rclcpp::SystemDefaultsQoS());

    // Create the servo object.
    planning_scene_monitor_ =
        createPlanningSceneMonitor(ptr_, servo_param_listener->get_params());
    servo_ = std::make_unique<Servo>(ptr_, servo_param_listener,
                                     planning_scene_monitor_);

    // Set the command type for servo.
    servo_->setCommandType(CommandType::TWIST);

    // Start loop:
    thread_ = std::thread(&MoveitController::servo_loop, this);
  }

private:
  DebugMethods debug_methods_;
  servo::Params servo_params_;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr
      trajectory_outgoing_cmd_pub_;
  std::shared_ptr<planning_scene_monitor::PlanningSceneMonitor>
      planning_scene_monitor_;
  std::unique_ptr<Servo> servo_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr
      twist_subscription_;
  bool published_ = false;

  TwistCommand command_{"fr3_link0", {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
  std::thread thread_;

  void callback(const geometry_msgs::msg::TwistStamped::SharedPtr msg) {
    command_.velocities = {msg->twist.linear.x,  msg->twist.linear.y,
                           msg->twist.linear.z,  msg->twist.angular.x,
                           msg->twist.angular.y, msg->twist.angular.z};
  }

  void servo_loop() {
    rclcpp::WallRate rate(1.0 / servo_params_.publish_period);

    auto robot_state =
        planning_scene_monitor_->getStateMonitor()->getCurrentState();
    auto joint_model_group =
        robot_state->getJointModelGroup(servo_params_.move_group_name);

    std::deque<KinematicState> joint_cmd_rolling_window;
    KinematicState current_state = servo_->getCurrentRobotState(true);
    updateSlidingWindow(current_state, joint_cmd_rolling_window,
                        servo_params_.max_expected_latency, this->now());

    while (rclcpp::ok()) {
      auto status = servo_->getStatus();

      if (status != StatusCode::INVALID) {
        auto joint_state = servo_->getNextJointState(robot_state, command_);
        updateSlidingWindow(joint_state, joint_cmd_rolling_window,
                            servo_params_.max_expected_latency, this->now());
        if (const auto msg = composeTrajectoryMessage(
                servo_params_, joint_cmd_rolling_window)) {
          trajectory_outgoing_cmd_pub_->publish(msg.value());
        }
        if (!joint_cmd_rolling_window.empty()) {
          robot_state->setJointGroupPositions(
              joint_model_group, joint_cmd_rolling_window.back().positions);
          robot_state->setJointGroupVelocities(
              joint_model_group, joint_cmd_rolling_window.back().velocities);
        }
      }
      rate.sleep();
    }
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  std::shared_ptr<MoveitController> node = std::make_shared<MoveitController>();
  node->initialize_moveit();
  rclcpp::spin(node);
  rclcpp::shutdown();
}