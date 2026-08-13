#include <array>
#include <memory>
#include <rclcpp/logging.hpp>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"

using sensor_msgs::msg::NavSatFix;

class NavSatCovarianceFixer : public rclcpp::Node {
 public:
  NavSatCovarianceFixer() : Node("navsat_covariance_fixer") {
    horizontal_stddev_ =
        this->declare_parameter<double>("horizontal_stddev", 0.5);
    vertical_stddev_ = this->declare_parameter<double>("vertical_stddev", 0.5);
    input_topic_ =
        this->declare_parameter<std::string>("input_topic", "gps/fix");
    output_topic_ =
        this->declare_parameter<std::string>("output_topic", "gps/fix_cov");

    const double h_var = horizontal_stddev_ * horizontal_stddev_;
    const double v_var = vertical_stddev_ * vertical_stddev_;
    covariance_ = {
        h_var, 0.0, 0.0, 0.0, h_var, 0.0, 0.0, 0.0, v_var,
    };

    publisher_ = this->create_publisher<NavSatFix>(output_topic_, 1);
    subscription_ = this->create_subscription<NavSatFix>(
        input_topic_, 1, [this](const sensor_msgs::msg::NavSatFix msg) {
          NavSatFix out = msg;
          out.position_covariance = covariance_;
          out.position_covariance_type =
              NavSatFix::COVARIANCE_TYPE_DIAGONAL_KNOWN;
          publisher_->publish(out);
        });

    RCLCPP_INFO(this->get_logger(),
                "Republishing '%s' -> '%s' with horizontal_stddev=%.3f m, "
                "vertical_stddev=%.3f m",
                input_topic_.c_str(), output_topic_.c_str(), horizontal_stddev_,
                vertical_stddev_);
  }

 private:
  double horizontal_stddev_;
  double vertical_stddev_;
  std::string input_topic_;
  std::string output_topic_;
  std::array<double, 9> covariance_;

  rclcpp::Publisher<NavSatFix>::SharedPtr publisher_;
  rclcpp::Subscription<NavSatFix>::SharedPtr subscription_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<NavSatCovarianceFixer>());
  rclcpp::shutdown();
  return 0;
}
