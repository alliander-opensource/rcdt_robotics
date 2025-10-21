// # SPDX-FileCopyrightText: Alliander N. V.
//
// # SPDX-License-Identifier: Apache-2.0

#include "rcdt_messages/srv/express_pose_in_other_frame.hpp"
#include "rcdt_messages/srv/transform_pose.hpp"
#include <gtest/gtest.h>
#include <iostream>
#include <rcdt_messages/srv/express_pose_in_other_frame.hpp>
#include <rcdt_messages/srv/transform_pose.hpp>
#include <rclcpp/executors.hpp>
#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <rclcpp/rclcpp.hpp>

class PackageTester : public rclcpp::Node {
public:
  PackageTester() : Node("manipulate_pose_tester") {
    express_pose_in_other_frame_client_ =
        this->create_client<rcdt_messages::srv::ExpressPoseInOtherFrame>(
            "~/express_pose_in_other_frame");
    transform_pose_client_ =
        this->create_client<rcdt_messages::srv::TransformPose>(
            "~/transform_pose");
  }

  bool waitForServices(std::chrono::seconds timeout) {
    bool express_pose_in_other_frame_available =
        express_pose_in_other_frame_client_->wait_for_service(timeout);
    if (!express_pose_in_other_frame_available) {
      RCLCPP_ERROR(this->get_logger(), "Service 'express_pose_in_other_frame' "
                                       "not available within timeout.");
      return false;
    }

    bool transform_pose_available =
        transform_pose_client_->wait_for_service(timeout);
    if (!transform_pose_available) {
      RCLCPP_ERROR(this->get_logger(),
                   "Service 'transform_pose' not available within timeout.");
      return false;
    }

    return true;
  }

private:
  rclcpp::Client<rcdt_messages::srv::ExpressPoseInOtherFrame>::SharedPtr
      express_pose_in_other_frame_client_;
  rclcpp::Client<rcdt_messages::srv::TransformPose>::SharedPtr
      transform_pose_client_;
};
