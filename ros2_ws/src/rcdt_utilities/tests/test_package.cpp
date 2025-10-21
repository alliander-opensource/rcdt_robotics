// # SPDX-FileCopyrightText: Alliander N. V.
//
// # SPDX-License-Identifier: Apache-2.0

#include "tests/test_package.hpp"
#include "rcdt_utilities/manipulate_pose.h"

/**
 * @brief Test fixture class for the PackageTester Node.
 * * This fixture ensures a clean setup and teardown for each test,
 * making sure the PackageTester has access to all the right data.
 */
class PackageTesterFixture : public testing::Test {
public:
  static void SetUpTestSuite() {
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }

    executor_ = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
  }

  static void TearDownTestSuite() {
    executor_.reset();
    rclcpp::shutdown();
  }

protected:
  void SetUp() override {
    tester_node_ = std::make_shared<PackageTester>();
    executor_->add_node(tester_node_);
  }

  void TearDown() override {
    executor_->remove_node(tester_node_);
    tester_node_.reset();
  }

  std::shared_ptr<PackageTester> tester_node_;
  static std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> executor_;
};

std::shared_ptr<rclcpp::executors::MultiThreadedExecutor>
    PackageTesterFixture::executor_ = nullptr;

// TESTS
TEST_F(PackageTesterFixture, TestTrue) {
  ASSERT_EQ(std::string(tester_node_->get_name()), "manipulate_pose_tester");
}

// MAIN
int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);

  int _argc = 0;
  const char **_argv = nullptr;

  rclcpp::init(_argc, _argv);

  return RUN_ALL_TESTS();
}
