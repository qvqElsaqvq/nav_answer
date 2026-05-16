//
// Created by elsa on 2026/5/15.
//

#include "robot_serial.h"

RobotSerial::RobotSerial() : Node("robot_serial_node") {
    declare_parameter("serial_name", "/dev/pts/3");
    serial = std::move(my_serial::MySerial(get_parameter("serial_name").as_string(), 115200));
    RCLCPP_INFO(this->get_logger(), "robot_serial init success");
    serial.registerCallback(1, [this](const my_serial::password_receive_t &msg) {
        robot_msgs::msg::SerialFullKey _fullKey;
        _fullKey.full_key = msg.password;
        FullKeyPublisher->publish(_fullKey);
    });
    FullKeyPublisher = create_publisher<robot_msgs::msg::SerialFullKey>("/serial/full_key_", 1);
    SegmentKeySubscriber = create_subscription<robot_msgs::msg::SerialSegmentKey>("/serial/segment_key_", 1,
        std::bind(&RobotSerial::SegmentKeyCallback, this, std::placeholders::_1));
    serial.spin(true);
}

void RobotSerial::SegmentKeyCallback(const robot_msgs::msg::SerialSegmentKey::SharedPtr msg) {
    my_serial::Head head;
    my_serial::Tail tail;
    my_serial::password_send_t password_send{
        (uint64_t) (msg->segment_key_1),
        (uint64_t) (msg->segment_key_2),
    };
    head.length = sizeof(my_serial::password_send_t);
    head.cmd_id = 0;
    serial.write(head, password_send, tail);
    RCLCPP_INFO(this->get_logger(), "serial write:%lu,%lu", msg->segment_key_1, msg->segment_key_2);
}
