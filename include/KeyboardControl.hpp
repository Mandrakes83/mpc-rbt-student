//
// Created by student on 26.3.25.
//

#ifndef KEYBOARDCONTROL_H
#define KEYBOARDCONTROL_H

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <termios.h>

class KeyboardControlNode : public rclcpp::Node {
public:
    KeyboardControlNode();
    ~KeyboardControlNode();

private:
    void timerCallback();

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;

    struct termios old_termios_;
};

#endif //KEYBOARDCONTROL_H
