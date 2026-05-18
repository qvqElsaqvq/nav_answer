//
// Created by elsa on 2026/5/17.
//

#ifndef ROBOT_BEHAVIOR_TREE_UPDATE_MAPINFO_HPP
#define ROBOT_BEHAVIOR_TREE_UPDATE_MAPINFO_HPP

#include <string>
#include <memory>
#include <mutex>

#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp_v3/action_node.h"

#include "robot_msgs/msg/map_info_msgs.hpp"
#include "robot_msgs/msg/map_info.hpp"

namespace nav2_behavior_tree
{

    enum TargetType : uint8_t {
        STAR = 0,
        BASE = 1,
        ENEMY_BASE = 2,
        PURPLEENTRY = 3,
        GREENENTRY = 4,
        SENTRY = 5,
        ENEMY = 6,
        PURPLEEXIT = 7,
        GREENEXIT = 8,
    };

    class UpdateMapinfoAction : public BT::SyncActionNode
    {
    public:
        /**
         * @brief A constructor for nav2_behavior_tree::IsBatteryLowCondition
         * @param action_name Name for the XML tag for this node
         * @param conf BT node configuration
         */
        UpdateMapinfoAction(
            const std::string &action_name,
            const BT::NodeConfiguration &conf);

        UpdateMapinfoAction() = delete;

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

    private:
        void mapInfoCallback(robot_msgs::msg::MapInfoMsgs::SharedPtr msg);

        rclcpp::Node::SharedPtr node_;
        rclcpp::CallbackGroup::SharedPtr callback_group_;
        rclcpp::executors::SingleThreadedExecutor callback_group_executor_;
        rclcpp::Subscription<robot_msgs::msg::MapInfoMsgs>::SharedPtr map_info_msgs_sub_;

        int sentry_HP_;
        int enemy_num_;
        bool is_transfering_;
        bool is_bullet_low_;

        std::vector<robot_msgs::msg::MapInfo> mapInfo_;
    };

} // namespace nav2_behavior_tree

#endif //ROBOT_BEHAVIOR_TREE_UPDATE_MAPINFO_HPP
