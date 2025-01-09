#include <moveit/kinematics_base/kinematics_base.hpp>
#include <moveit/moveit_cpp/moveit_cpp.hpp>
#include <moveit/moveit_cpp/planning_component.hpp>
#include <moveit/robot_model/joint_model_group.hpp>
#include <moveit/robot_model/robot_model.hpp>
#include <rclcpp/node.hpp>

class MoveitCpp {
public:
  void initialize(std::shared_ptr<rclcpp::Node> node);
  bool move_to_configuration(std::string configuration);
  void plan_and_execute();

private:
  std::shared_ptr<rclcpp::Node> node;
  std::string planning_group;
  moveit_cpp::MoveItCppPtr moveit_cpp;
  moveit_cpp::PlanningComponentPtr planning_components;
  moveit::core::RobotModelConstPtr robot_model;
  const moveit::core::JointModelGroup *joint_model_group;
};