//
// Created by elsa on 2026/5/17.
//

#include "robot_behavior_tree/plugins/action/update_mapinfo.hpp"

namespace nav2_behavior_tree {
    UpdateMapinfoAction::UpdateMapinfoAction(
        const std::string &action_name,
        const BT::NodeConfiguration &conf)
        : BT::SyncActionNode(action_name, conf),
          sentry_HP_(0), enemy_num_(2), is_transfering_(false), is_bullet_low_(false) {
        node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
        callback_group_ = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive, false);
        callback_group_executor_.add_callback_group(callback_group_, node_->get_node_base_interface());

        rclcpp::SubscriptionOptions sub_option;
        sub_option.callback_group = callback_group_;
        map_info_msgs_sub_ = node_->create_subscription<robot_msgs::msg::MapInfoMsgs>("/map_info",
            rclcpp::SystemDefaultsQoS(),
            std::bind(&UpdateMapinfoAction::mapInfoCallback, this, std::placeholders::_1), sub_option);

        config().blackboard->set<int>("sentry_HP", sentry_HP_);
        config().blackboard->set<int>("enemy_num", enemy_num_);
        config().blackboard->set<bool>("is_bullet_low", is_bullet_low_);
        config().blackboard->set<bool>("is_transfering", is_transfering_);

        config().blackboard->set<bool>("is_sentry_exist", false);
        config().blackboard->set<bool>("is_sentry_out_of_center", true);
        config().blackboard->set<double>("sentry_pos_x", 0.0);
        config().blackboard->set<double>("sentry_pos_y", 0.0);
        config().blackboard->set<double>("sentry_pos_theta", 0.0);
        config().blackboard->set<bool>("is_base_exist", false);
        config().blackboard->set<double>("base_x", 0.0);
        config().blackboard->set<double>("base_y", 0.0);

        robot_msgs::msg::MapInfo temp;
        temp.is_exist = false;
        temp.is_out_of_center = false;
        temp.pos.x = 1000.0;
        temp.pos.y = 1000.0;
        temp.pos.theta = 0.0;
        for (int i = 0; i < 9; ++i) {
            mapInfo_.push_back(temp);
        }
    }

    BT::NodeStatus UpdateMapinfoAction::tick() {
        callback_group_executor_.spin_some();

        config().blackboard->set<int>("sentry_HP", sentry_HP_);
        RCLCPP_INFO(node_->get_logger(), "sentry_HP_=%d", sentry_HP_);
        config().blackboard->set<int>("enemy_num", enemy_num_);
        config().blackboard->set<bool>("is_bullet_low", is_bullet_low_);
        config().blackboard->set<bool>("is_transfering", is_transfering_);

        config().blackboard->set<bool>("is_sentry_exist", mapInfo_[SENTRY].is_exist);
        config().blackboard->set<bool>("is_sentry_out_of_center", mapInfo_[SENTRY].is_out_of_center);
        config().blackboard->set<double>("sentry_pos_x", mapInfo_[SENTRY].pos.x);
        config().blackboard->set<double>("sentry_pos_y", mapInfo_[SENTRY].pos.y);
        config().blackboard->set<double>("sentry_pos_theta", mapInfo_[SENTRY].pos.theta);
        config().blackboard->set<bool>("is_base_exist", mapInfo_[BASE].is_exist);
        config().blackboard->set<double>("base_x", mapInfo_[BASE].pos.x);
        config().blackboard->set<double>("base_y", mapInfo_[BASE].pos.y);

        return BT::NodeStatus::SUCCESS;
    }

    void UpdateMapinfoAction::mapInfoCallback(robot_msgs::msg::MapInfoMsgs::SharedPtr msg) {
        // RCLCPP_INFO(node_->get_logger(), "Received map info message");
        sentry_HP_ = msg->sentry_hp;
        // RCLCPP_INFO(node_->get_logger(), "receive sentry_HP_=%d", sentry_HP_);
        enemy_num_ = msg->enemy_num;
        is_bullet_low_ = msg->is_bullet_low;
        is_transfering_ = msg->is_transfering;

        for (int i = 0; i < 9; i++) {
            mapInfo_[i] = msg->map_info[i];
        }
    }
} // namespace nav2_behavior_tree
#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory) {
    factory.registerNodeType<nav2_behavior_tree::UpdateMapinfoAction>("UpdateMapinfo");
}
