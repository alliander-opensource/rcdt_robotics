#include <rclcpp/rclcpp.hpp>
#include <robot_localization/srv/set_datum.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/nav_sat_status.hpp>

#include "sensor_msgs/msg/nav_sat_fix.hpp"

using namespace std::chrono_literals;

class DatumGateNode : public rclcpp::Node {
 public:
  DatumGateNode() : Node("datum_gate_node") {
    gps_topic_ = declare_parameter<std::string>("gps_topic", "/gps/fix");
    datum_service_ = declare_parameter<std::string>("datum_service", "/datum");
    required_consecutive_fixes_ =
        declare_parameter<int>("required_consecutive_fixes", 5);
    max_position_covariance_ =
        declare_parameter<double>("max_position_covariance_m2", 4.0);

    datum_client_ =
        create_client<robot_localization::srv::SetDatum>(datum_service_);

    fix_sub_ = create_subscription<sensor_msgs::msg::NavSatFix>(
        gps_topic_, rclcpp::SensorDataQoS(),
        [this](const sensor_msgs::msg::NavSatFix::SharedPtr msg) {
          this->fixCallback(msg);
        });

    RCLCPP_INFO(
        get_logger(),
        "Waiting for %d consecutive good fixes on '%s' before setting datum",
        required_consecutive_fixes_, gps_topic_.c_str());
  }

 private:
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

  void setDatum(const sensor_msgs::msg::NavSatFix& msg) {
    if (!datum_client_->wait_for_service(1s)) {
      RCLCPP_WARN(get_logger(),
                  "'%s' service not up yet, will retry on next good fix",
                  datum_service_.c_str());
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
          RCLCPP_INFO(get_logger(),
                      "Datum set, gate satisfied - shutting down");
          rclcpp::shutdown();
        });
  }

  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr fix_sub_;
  rclcpp::Client<robot_localization::srv::SetDatum>::SharedPtr datum_client_;

  std::string gps_topic_;
  std::string datum_service_;
  int required_consecutive_fixes_;
  double max_position_covariance_;
  double heading_settle_time_s_ = 15.0;
  int consecutive_good_fixes_ = 0;
  bool datum_requested_ = false;
  std::optional<rclcpp::Time> start_time_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DatumGateNode>());
  rclcpp::shutdown();
  return 0;
}
