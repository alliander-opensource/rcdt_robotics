// # SPDX-FileCopyrightText: Alliander N. V.
//
// # SPDX-License-Identifier: Apache-2.0

#include <memory>
#include <rclcpp/logging.hpp>
#include <rclcpp/timer.hpp>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "rclcpp/rclcpp.hpp"

class GoalProjectorNode : public rclcpp::Node {
 public:
  GoalProjectorNode() : Node("goal_projector_node") {
    margin_ = declare_parameter<double>("margin", 0.5);
    max_cost_valid_goal_ =
        declare_parameter<uint8_t>("max_cost_valid_goal", 252);

    costmap_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
        "/costmap_topic", 1,
        [this](const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
          costmap_ = msg;
          projectAndPublish();
        });
    goal_pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
        "/goal_pose_topic", 1,
        [this](const geometry_msgs::msg::PoseStamped msg) {
          if (costmap_ == nullptr) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                                 "No costmap received yet. Ignoring goal.");
            return;
          }

          if (msg.header.frame_id != costmap_->header.frame_id) {
            RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 2000,
                                  "Goal frame ID %s does not match costmap "
                                  "frame ID %s. Ignoring goal.",
                                  msg.header.frame_id.c_str(),
                                  costmap_->header.frame_id.c_str());
            return;
          }

          true_goal_ = msg;
        });

    updated_goal_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(
        "/updated_goal_pose_topic", 1);

    RCLCPP_INFO(get_logger(), "Goal projector node started.");
  }

 private:
  void projectAndPublish() {
    // return if costmap is not received yet
    if (costmap_ == nullptr) {
      return;
    }

    // check if goal is within costmap bounds
    auto updated_goal = true_goal_;
    if (!isWithinBounds(true_goal_)) {
      updated_goal = projectGoal();
    }

    updated_goal.header.stamp = now();
    updated_goal_pub_->publish(updated_goal);
  }

  geometry_msgs::msg::PoseStamped projectGoal() {
    const auto& m = costmap_->info;

    // current position is midpoint of costmap
    const double current_x = m.origin.position.x + m.resolution * m.width / 2.0;
    const double current_y =
        m.origin.position.y + m.resolution * m.height / 2.0;

    // maximum distance and
    const double max_x = m.resolution * m.width / 2.0 - margin_;
    const double max_y = m.resolution * m.height / 2.0 - margin_;

    const double dx = true_goal_.pose.position.x - current_x;
    const double dy = true_goal_.pose.position.y - current_y;

    // multiply distance by factor
    double scale = 1.0;
    if (std::abs(dx) > 1e-9) scale = std::min(scale, max_x / std::abs(dx));
    if (std::abs(dy) > 1e-9) scale = std::min(scale, max_y / std::abs(dy));
    scale = std::max(scale, 0.0);

    geometry_msgs::msg::PoseStamped projected_goal;
    projected_goal.header.frame_id = costmap_->header.frame_id;
    projected_goal.pose.position.x = current_x + dx * scale;
    projected_goal.pose.position.y = current_y + dy * scale;

    // find first valid cost cell along line
    while (!isValidGoal(projected_goal)) {
      scale -= m.resolution / std::hypotf(dx, dy);
      projected_goal.pose.position.x = current_x + dx * scale;
      projected_goal.pose.position.y = current_y + dy * scale;
    }
    return projected_goal;
  }

  bool isWithinBounds(const geometry_msgs::msg::PoseStamped& msg) {
    // check if goal xy is within costmap origin + costmap size + margin_
    // note origin of costmap is at bottom-left
    const auto& m = costmap_->info;
    const double ox = m.origin.position.x;
    const double oy = m.origin.position.y;
    const double x = msg.pose.position.x;
    const double y = msg.pose.position.y;
    return x > ox + margin_ && x < ox + m.width * m.resolution - margin_ &&
           y > oy + margin_ && y < oy + m.height * m.resolution - margin_;
  }

  bool isValidGoal(const geometry_msgs::msg::PoseStamped& goal) {
    // get cell index of goal
    const auto& m = costmap_->info;
    int mx = static_cast<int>((goal.pose.position.x - m.origin.position.x) /
                              m.resolution);
    int my = static_cast<int>((goal.pose.position.y - m.origin.position.y) /
                              m.resolution);
    size_t index = my * m.width + mx;
    uint8_t value = costmap_->data[index];

    // get costmap value at cell
    return value < max_cost_valid_goal_;
  }

  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr
      goal_pose_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr
      updated_goal_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  nav_msgs::msg::OccupancyGrid::ConstSharedPtr costmap_{nullptr};
  geometry_msgs::msg::PoseStamped true_goal_;

  float margin_;
  uint8_t max_cost_valid_goal_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GoalProjectorNode>());
  rclcpp::shutdown();
  return 0;
}
