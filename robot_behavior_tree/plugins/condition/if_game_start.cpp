//
// Created by elsa on 2026/5/17.
//

#include <string>

#include "robot_behavior_tree/plugins/condition/if_game_start.hpp"

namespace nav2_behavior_tree {
    IfGameStartCondition::IfGameStartCondition(
        const std::string &condition_name,
        const BT::NodeConfiguration &conf)
        : BT::ConditionNode(condition_name, conf) {
        node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");

        config().blackboard->get<int>("sentry_HP", sentry_hp_);
    }

    BT::NodeStatus IfGameStartCondition::tick() {
        config().blackboard->get<int>("sentry_HP", sentry_hp_);
        if (sentry_hp_ > 0) {
            return BT::NodeStatus::SUCCESS;
        }
        RCLCPP_INFO(node_->get_logger(), "等待游戏开始");
        return BT::NodeStatus::FAILURE;
    }
} // namespace nav2_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory) {
    factory.registerNodeType<nav2_behavior_tree::IfGameStartCondition>("IfGameStart");
}
