//
// Created by elsa on 2026/5/17.
//

#ifndef ROBOT_BEHAVIOR_TREE_NAV_VEL_REMAP_HPP
#define ROBOT_BEHAVIOR_TREE_NAV_VEL_REMAP_HPP

#include <string>
#include <memory>
#include <mutex>

#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp_v3/action_node.h"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose2_d.hpp"

namespace nav2_behavior_tree
{

    class NavVelRemapAction : public BT::SyncActionNode
    {
    public:
        /**
         * @brief A constructor for nav2_behavior_tree::IsBatteryLowCondition
         * @param action_name Name for the XML tag for this node
         * @param conf BT node configuration
         */
        NavVelRemapAction(
            const std::string &action_name,
            const BT::NodeConfiguration &conf);

        NavVelRemapAction() = delete;

        /**
         * @brief The main override required by a BT action
         * @return BT::NodeStatus Status of tick execution
         */
        BT::NodeStatus tick() override;

        /**
         * @brief Creates list of BT ports
         * @return BT::PortsList Containing node-specific ports
         */
        static BT::PortsList providedPorts()
        {
            return {};
        }

        void navVelCallback(geometry_msgs::msg::Twist::SharedPtr msg);

    private:
        rclcpp::Node::SharedPtr node_;
        rclcpp::CallbackGroup::SharedPtr callback_group_;
        rclcpp::executors::SingleThreadedExecutor callback_group_executor_;
        rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr nav_vel_sub_;

        rclcpp::Publisher<geometry_msgs::msg::Pose2D>::SharedPtr pose_pub_;

        geometry_msgs::msg::Pose2D control_pose_;
    };

} // namespace nav2_behavior_tree

#endif //ROBOT_BEHAVIOR_TREE_NAV_VEL_REMAP_HPP
