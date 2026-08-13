// # SPDX-FileCopyrightText: Alliander N. V.
//
// # SPDX-License-Identifier: Apache-2.0

#include <array>
#include <memory>
#include <rclcpp/logging.hpp>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"

using sensor_msgs::msg::NavSatFix;

/**
 * @brief A ROS2 node that republishes NavSatFix messages with a fixed
 * diagonal position covariance.
 *
 * Gazebo's navsat sensor publishes NavSatFix with position_covariance
 * left at all zeros / COVARIANCE_TYPE_UNKNOWN, since the underlying
 * gz.msgs.NavSat message carries no covariance data. This node fills
 * that gap so downstream consumers (e.g. a datum gate node or
 * navsat_transform_node) have a covariance to check against.
 */
class NavSatCovarianceFixer : public rclcpp::Node {
 public:
  /**
   * @brief Constructor for the NavSatCovarianceFixer class.
   *
   * Declares the horizontal_stddev and vertical_stddev parameters,
   * derives the fixed covariance matrix from them, and sets up the
   * subscription/publisher pair.
   */
  NavSatCovarianceFixer() : Node("navsat_covariance_fixer") {
    horizontal_stddev_ =
        this->declare_parameter<double>("horizontal_stddev", 0.5);
    vertical_stddev_ = this->declare_parameter<double>("vertical_stddev", 0.5);

    covariance_ = computeCovariance(horizontal_stddev_, vertical_stddev_);

    publisher_ = this->create_publisher<NavSatFix>("gps/fix_out", 1);
    subscription_ = this->create_subscription<NavSatFix>(
        "gps/fix_in", 1, [this](const sensor_msgs::msg::NavSatFix msg) {
          NavSatFix out = msg;
          out.position_covariance = covariance_;
          out.position_covariance_type =
              NavSatFix::COVARIANCE_TYPE_DIAGONAL_KNOWN;
          publisher_->publish(out);
        });

    RCLCPP_INFO(
        this->get_logger(),
        "Republishing GPS NavSatFix data with horizontal_stddev=%.3f m, "
        "vertical_stddev=%.3f m",
        horizontal_stddev_, vertical_stddev_);
  }

 private:
  /**
   * @brief Builds a diagonal ENU position covariance matrix from stddevs.
   *
   * @param h_stddev Horizontal (East/North) standard deviation, in meters.
   * @param v_stddev Vertical (Up) standard deviation, in meters.
   * @return std::array<double, 9> Row-major 3x3 covariance, off-diagonals
   * zero.
   */
  static std::array<double, 9> computeCovariance(double h_stddev,
                                                 double v_stddev) {
    const double h_var = h_stddev * h_stddev;
    const double v_var = v_stddev * v_stddev;
    return {h_var, 0.0, 0.0, 0.0, h_var, 0.0, 0.0, 0.0, v_var};
  }

  /// Horizontal (East/North) standard deviation, in meters
  double horizontal_stddev_;
  /// Vertical (Up) standard deviation, in meters
  double vertical_stddev_;
  /// Fixed diagonal position covariance derived from the stddev parameters
  std::array<double, 9> covariance_;

  /// Publisher for the covariance-stamped NavSatFix output
  rclcpp::Publisher<NavSatFix>::SharedPtr publisher_;
  /// Subscriber for the raw, covariance-less NavSatFix input
  rclcpp::Subscription<NavSatFix>::SharedPtr subscription_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<NavSatCovarianceFixer>());
  rclcpp::shutdown();
  return 0;
}
