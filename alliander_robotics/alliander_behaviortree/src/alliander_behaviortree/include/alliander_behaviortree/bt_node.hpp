#include <behaviortree_cpp/basic_types.h>

#include "alliander_interfaces/action/trigger_action.hpp"
#include "behaviortree_ros2/bt_action_node.hpp"

using Trigger = alliander_interfaces::action::TriggerAction;
using GoalHandleFibonacci = rclcpp_action::ServerGoalHandle<Trigger>;

class RosBehaviorTree : public BT::RosActionNode<Trigger> {
 public:
  RosBehaviorTree(const std::string& name, const BT::NodeConfig& conf,
                  const BT::RosNodeParams& params)
      : RosActionNode<Trigger>(name, conf, params) {};

  static BT::PortsList providedPorts() {
    return providedBasicPorts({BT::InputPort<unsigned>("order")});
  }

  bool setGoal(RosActionNode::Goal& goal) override {
    getInput("order", goal.structure_needs_at_least_one_member);
  }
};
