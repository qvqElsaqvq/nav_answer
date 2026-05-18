//
// Created by elsa on 2026/5/17.
//

#include "robot_behavior_tree/plugins/action/go_base.hpp"

namespace nav2_behavior_tree {
    GoBaseAction::GoBaseAction(
        const std::string &action_name,
        const BT::NodeConfiguration &conf)
        : BT::SyncActionNode(action_name, conf),
          base_x_(0.0), base_y_(0.0), is_base_exist_(false) {
        node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
        goal_pub_ = node_->create_publisher<geometry_msgs::msg::PoseStamped>("/goal_pose", 10);

        goal_pose.header.stamp = node_->now();
        goal_pose.header.frame_id = "map";
        goal_pose.pose.position.x = 0.0;
        goal_pose.pose.position.y = 0.0;
        goal_pose.pose.position.z = 0.0;
        goal_pose.pose.orientation.x = 0.0;
        goal_pose.pose.orientation.y = 0.0;
        goal_pose.pose.orientation.z = 0.0;
        goal_pose.pose.orientation.w = 1.0;

        config().blackboard->get<bool>("is_base_exist", is_base_exist_);
        config().blackboard->get<double>("base_x", base_x_);
        config().blackboard->get<double>("base_y", base_y_);
    }

    BT::NodeStatus GoBaseAction::tick() {
        config().blackboard->get<bool>("is_base_exist", is_base_exist_);
        config().blackboard->get<double>("base_x", base_x_);
        config().blackboard->get<double>("base_y", base_y_);
        RCLCPP_INFO(node_->get_logger(), "is_base_exist: %d", is_base_exist_);

        if (is_base_exist_) {
            goal_pose.header.stamp = node_->now();
            goal_pose.header.frame_id = "map";
            goal_pose.pose.position.x = base_x_;
            goal_pose.pose.position.y = base_y_;
            goal_pose.pose.position.z = 0.0;
            goal_pose.pose.orientation.x = 0.0;
            goal_pose.pose.orientation.y = 0.0;
            goal_pose.pose.orientation.z = 0.0;
            goal_pose.pose.orientation.w = 1.0;
            goal_pub_->publish(goal_pose);
            RCLCPP_INFO(node_->get_logger(), "go base: x=%lf, y=%lf", base_x_, base_y_);
        }
        return BT::NodeStatus::SUCCESS;
    }
} // namespace nav2_behavior_tree
#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory) {
    factory.registerNodeType<nav2_behavior_tree::GoBaseAction>("GoBase");
}
