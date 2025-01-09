#include <geometry_msgs/msg/twist_stamped.hpp>
#include <memory>
#include <moveit/kinematics_base/kinematics_base.hpp>
#include <moveit/moveit_cpp/planning_component.hpp>
#include <moveit/planning_scene/planning_scene.hpp>
#include <moveit/planning_scene_monitor/planning_scene_monitor.hpp>
#include <moveit/robot_state/robot_state.hpp>
#include <moveit_servo/servo.hpp>
#include <rclcpp/node.hpp>

class MoveitServo {
public:
  void initialize(
      std::shared_ptr<rclcpp::Node> node,
      planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor);
  void activate();
  void deactivate();

private:
  std::shared_ptr<rclcpp::Node> node;
  planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr pub;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr sub;
  servo::Params params;
  std::unique_ptr<moveit_servo::Servo> servo;
  moveit::core::RobotStatePtr robot_state;
  const moveit::core::JointModelGroup *joint_model_group;
  moveit_servo::TwistCommand command;
  std::deque<moveit_servo::KinematicState> joint_cmd_rolling_window;
  std::thread thread;
  bool active;
  void servo_loop();
  void update_robot_state();
  void callback(const geometry_msgs::msg::TwistStamped::SharedPtr msg);
};