#include "moveit_cpp.hpp"

void MoveitCpp::initialize(std::shared_ptr<rclcpp::Node> node) {

  planning_group = "fr3_arm";

  moveit_cpp = std::make_shared<moveit_cpp::MoveItCpp>(node);
  moveit_cpp->getPlanningSceneMonitorNonConst()->providePlanningSceneService();

  planning_components = std::make_shared<moveit_cpp::PlanningComponent>(
      planning_group, moveit_cpp);
  robot_model = moveit_cpp->getRobotModel();
  joint_model_group = robot_model->getJointModelGroup(planning_group);
};

bool MoveitCpp::move_to_configuration(std::string configuration) {
  planning_components->setGoal(configuration);
  auto plan = planning_components->plan();
  moveit_cpp->execute(plan.trajectory);
  return true;
};

void MoveitCpp::plan_and_execute() {
  planning_components->setStartStateToCurrentState();
  geometry_msgs::msg::PoseStamped target_pose1;
  target_pose1.header.frame_id = "fr3_link0";
  target_pose1.pose.orientation.w = 1.0;
  target_pose1.pose.position.x = 0.28;
  target_pose1.pose.position.y = -0.2;
  target_pose1.pose.position.z = 0.5;
  planning_components->setGoal(target_pose1, "fr3_hand");

  auto plan = planning_components->plan();

  moveit_cpp->execute(plan.trajectory);
}