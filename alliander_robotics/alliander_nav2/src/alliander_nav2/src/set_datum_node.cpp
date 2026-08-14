// # SPDX-FileCopyrightText: Alliander N. V.
//
// # SPDX-License-Identifier: Apache-2.0

#include <rclcpp/rclcpp.hpp>
#include <robot_localization/srv/set_datum.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/nav_sat_status.hpp>

#include "sensor_msgs/msg/nav_sat_fix.hpp"

using namespace std::chrono_literals;

/**
 * @brief A ROS2 node that gates the robot_localization datum on a run of
 * consecutive, sufficiently-precise GPS fixes.
 *
 * Listens on the gps/fix topic and, once required_consecutive_fixes_ good
 * fixes in a row have been seen (fix status at or above STATUS_FIX, known
 * covariance, and horizontal covariance within max_position_covariance_),
 * calls robot_localization's datum service with that fix and shuts down.
 */
class DatumGateNode : public rclcpp::Node {
 public:
  /**
   * @brief Constructor for the DatumGateNode class.
   *
   * Sets up the NavSatFix subscription and the datum service client, then
   * declares the required_consecutive_fixes and max_position_covariance_m2
   * parameters.
   */
  DatumGateNode()
      : Node("set_datum_node"),
        fix_sub_(create_subscription<sensor_msgs::msg::NavSatFix>(
            "gps/fix", rclcpp::SensorDataQoS(),
            [this](const sensor_msgs::msg::NavSatFix::SharedPtr msg) {
              this->fixCallback(msg);
            })),
        datum_client_(
            create_client<robot_localization::srv::SetDatum>("datum")) {
    required_consecutive_fixes_ =
        declare_parameter<int>("required_consecutive_fixes", 5);
    max_position_covariance_ =
        declare_parameter<double>("max_position_covariance_m2", 4.0);

    RCLCPP_INFO(get_logger(),
                "Waiting for %d consecutive good fixes on before setting datum",
                required_consecutive_fixes_);
  }

 private:
  /**
   * @brief Callback for the GPS fix subscription.
   *
   * Tracks the streak of consecutive good fixes, resetting it whenever a
   * fix falls out of tolerance, and triggers setDatum() once the streak
   * reaches required_consecutive_fixes_. No-ops once a datum has already
   * been requested.
   *
   * @param msg Shared pointer to the incoming NavSatFix message.
   */
  void fixCallback(const sensor_msgs::msg::NavSatFix::SharedPtr msg) {
    if (datum_requested_) {
      return;
    }

    if (!isGoodFix(*msg)) {
      if (consecutive_good_fixes_ > 0) {
        RCLCPP_WARN(get_logger(),
                    "Fix dropped out of tolerance, resetting streak");
      }
      consecutive_good_fixes_ = 0;
      return;
    }

    ++consecutive_good_fixes_;
    RCLCPP_INFO(get_logger(), "Good fix %d/%d (lat=%.7f lon=%.7f status=%d)",
                consecutive_good_fixes_, required_consecutive_fixes_,
                msg->latitude, msg->longitude, msg->status.status);

    if (consecutive_good_fixes_ < required_consecutive_fixes_) {
      return;
    }

    setDatum(*msg);
  }

  /**
   * @brief Checks whether a fix meets the status and covariance
   * requirements to count toward the good-fix streak.
   *
   * @param msg NavSatFix message to evaluate.
   * @return bool True if the fix has at least STATUS_FIX, a known
   * covariance type, and horizontal covariance within
   * max_position_covariance_.
   */
  bool isGoodFix(const sensor_msgs::msg::NavSatFix& msg) const {
    // Require a fix
    if (msg.status.status < sensor_msgs::msg::NavSatStatus::STATUS_FIX) {
      return false;
    }

    // Require a covariance
    if (msg.position_covariance_type ==
        sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_UNKNOWN) {
      return false;
    }

    // Covariance check
    double horizontal_cov =
        std::max(msg.position_covariance[0], msg.position_covariance[4]);
    if (horizontal_cov <= 0.0 || horizontal_cov > max_position_covariance_) {
      return false;
    }

    // TODO add RTK-fix condition?
    return true;
  }

  /**
   * @brief Sends the SetDatum request for the given fix and shuts the node
   * down once it completes.
   *
   * If the datum service isn't up yet, resets the good-fix streak so the
   * gate re-accumulates fixes before retrying. Sets datum_requested_ before
   * the async call returns so fixCallback() stops processing further fixes
   * immediately.
   *
   * @param msg The qualifying NavSatFix to use as the datum.
   */
  void setDatum(const sensor_msgs::msg::NavSatFix& msg) {
    if (!datum_client_->wait_for_service(1s)) {
      RCLCPP_WARN(get_logger(),
                  "datum service not up yet, will retry on next good fix");
      consecutive_good_fixes_ = 0;
      return;
    }

    auto request =
        std::make_shared<robot_localization::srv::SetDatum::Request>();
    request->geo_pose.position.latitude = msg.latitude;
    request->geo_pose.position.longitude = msg.longitude;
    request->geo_pose.position.altitude = msg.altitude;
    request->geo_pose.orientation.w = 1.0;

    datum_requested_ = true;
    datum_client_->async_send_request(
        request,
        [this](
            rclcpp::Client<robot_localization::srv::SetDatum>::SharedFuture) {
          RCLCPP_INFO(get_logger(), "Datum set, shutting down");
          rclcpp::shutdown();
        });
  }

  /// Subscriber that listens to the GPS fix topic
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr fix_sub_;
  /// Client for robot_localization's SetDatum service
  rclcpp::Client<robot_localization::srv::SetDatum>::SharedPtr datum_client_;

  /// Number of consecutive good fixes required before setting the datum
  int required_consecutive_fixes_;
  /// Maximum acceptable horizontal position covariance, in m^2
  double max_position_covariance_;
  /// Current length of the consecutive good-fix streak
  int consecutive_good_fixes_ = 0;
  /// Whether a datum has already been requested (gate satisfied)
  bool datum_requested_ = false;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DatumGateNode>());
  rclcpp::shutdown();
  return 0;
}
