#include "moveit_controller.hpp"
#include <rcdt_utilities_msgs/srv/detail/move_to_configuration__struct.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/rclcpp.hpp>

using std::placeholders::_1;
using std::placeholders::_2;

MoveitController::MoveitController(rclcpp::NodeOptions options)
    : Node("moveit_controller", options) {};

void MoveitController::initialize() {
  auto node = shared_from_this();
  moveit_cpp.initialize(node);

  auto planning_scene_monitor = moveit_cpp.get_planning_scene_monitor();
  planning_scene.initialize(node, planning_scene_monitor);
  moveit_servo.initialize(node, planning_scene_monitor);

  service = create_service<MoveToConf>(
      "~/move_to_configuration",
      std::bind(&MoveitController::move_to_configuration, this, _1, _2));
}

void MoveitController::move_to_configuration(
    const std::shared_ptr<MoveToConf::Request> request,
    std::shared_ptr<MoveToConf::Response> response) {
  moveit_servo.deactivate();
  response->success = moveit_cpp.move_to_configuration(request->configuration);
  moveit_servo.activate();
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);
  auto node = std::make_shared<MoveitController>(node_options);
  node->initialize();
  rclcpp::spin(node);
  rclcpp::shutdown();
}