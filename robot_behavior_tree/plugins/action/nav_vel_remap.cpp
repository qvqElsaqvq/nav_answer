//
// Created by elsa on 2026/5/17.
//

#include "robot_behavior_tree/plugins/action/nav_vel_remap.hpp"

namespace nav2_behavior_tree
{
    NavVelRemapAction::NavVelRemapAction(
        const std::string &action_name,
        const BT::NodeConfiguration &conf)
        : BT::SyncActionNode(action_name, conf)
    {
        node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
        callback_group_ = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive, false);
        callback_group_executor_.add_callback_group(callback_group_, node_->get_node_base_interface());

        rclcpp::SubscriptionOptions sub_option;
        sub_option.callback_group = callback_group_;
        nav_vel_sub_ = node_->create_subscription<geometry_msgs::msg::Twist>("/cmd_vel",
            rclcpp::SystemDefaultsQoS(),
            std::bind(&NavVelRemapAction::navVelCallback, this, std::placeholders::_1), sub_option);

        pose_pub_ = node_->create_publisher<geometry_msgs::msg::Pose2D>("/pose",1);

        control_pose_.x = 0.0;
        control_pose_.y = 0.0;
        control_pose_.theta = 0.0;
    }

    BT::NodeStatus NavVelRemapAction::tick()
    {
        callback_group_executor_.spin_some();

        return BT::NodeStatus::SUCCESS;
    }

    void NavVelRemapAction::navVelCallback(geometry_msgs::msg::Twist::SharedPtr msg) {
        RCLCPP_INFO(node_->get_logger(), "-------------------");
        control_pose_.x = msg->linear.x * 0.1;
        control_pose_.y = -msg->linear.y * 0.1;
        control_pose_.theta = 0.0;
        pose_pub_->publish(control_pose_);
        RCLCPP_INFO(node_->get_logger(), "pub vel: x=%lf, y=%lf", control_pose_.x, control_pose_.y);
    }

} // namespace nav2_behavior_tree
#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
    factory.registerNodeType<nav2_behavior_tree::NavVelRemapAction>("NavVelRemap");
}