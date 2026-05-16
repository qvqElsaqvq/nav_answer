#ifndef ROBOT_SERIAL_H
#define ROBOT_SERIAL_H

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>

#include "robot_msgs/msg/serial_full_key.hpp"
#include "robot_msgs/msg/serial_segment_key.hpp"

#include "msg_serialize.h"
#include "serialPro.h"

namespace my_serial {
    message_data Head {
        uint64_t SOF = 0xAA; //头校验
        uint64_t length = 0; //长度
        uint64_t cmd_id = 0x00; //命令字
    };

    message_data password_send_t {
        uint64_t password1; //密码片段 
        uint64_t password2;
    };

    message_data password_receive_t {
        int64_t password; //密码
    };

    message_data Tail {
        uint64_t crc16 = 0xBB; //尾校验
    };

    class MySerial : public sp::serialPro<Head, Tail> {
    public:
        MySerial() = default;

        MySerial(std::string s, int band) : sp::serialPro<Head, Tail>(s, band) {
            registerChecker([](const Head &head)-> int { return head.SOF != 0xAA; }); //头校验
            registerChecker([](const Tail &tail, const uint8_t *, const int &)-> int { return tail.crc16 != 0xBB; });
            setGetId([](const Head &head)-> int { return head.cmd_id; }); //返回命令字
            setGetLength([](const Head &head)-> int { return (int) head.length; }); //返回长度
        }
    };
}

class RobotSerial : public rclcpp::Node {
private:
    my_serial::MySerial serial;
    rclcpp::Clock rosClock;
    rclcpp::Publisher<robot_msgs::msg::SerialFullKey>::SharedPtr FullKeyPublisher;
    rclcpp::Subscription<robot_msgs::msg::SerialSegmentKey>::SharedPtr SegmentKeySubscriber;

public:
    explicit RobotSerial();

    void SegmentKeyCallback(const robot_msgs::msg::SerialSegmentKey::SharedPtr msg);
};
#endif //RDBOT_SERIAL_H
