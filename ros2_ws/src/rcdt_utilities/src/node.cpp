#include "rcdt_utilities/manipulate_pose.h"
#include <rclcpp/executors/single_threaded_executor.hpp>

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  rclcpp::executors::SingleThreadedExecutor executor;
  auto node = std::make_shared<PoseManipulator>();
  executor.add_node(node);

  executor.spin();

  rclcpp::shutdown();
  return 0;
}
