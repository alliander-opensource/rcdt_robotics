#include "tests/test_package.h"
#include "rcdt_utilities/manipulate_pose.h"

#include <gtest/gtest.h>
#include <rclcpp/executors/single_threaded_executor.hpp>

TEST(PackageTest, Initialization) {
  rclcpp::executors::SingleThreadedExecutor executor;
  auto node = std::make_shared<PoseManipulator>();

  executor.spin_node_once(node);
  rclcpp::shutdown();
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);

  int _argc = 0;
  const char **_argv = nullptr;

  rclcpp::init(_argc, _argv);

  return RUN_ALL_TESTS();
}
