// Copyright 2016 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float32.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

/* This example creates a subclass of Node and uses std::bind() to register a
 * member function as a callback from the timer. */

class MinimalPublisher : public rclcpp::Node
{
public:
  MinimalPublisher()
  : Node("publisher"), count_(0)
  {
    this->declare_parameter<int>("Max",42);
    this->declare_parameter<int>("Min",32);
    publisher_ = this->create_publisher<std_msgs::msg::Float32>("battery_percentage", 10);
    subscription_ = this->create_subscription<std_msgs::msg::Float32>(
      "battery_voltage", 10, std::bind(&MinimalPublisher::topic_callback, this, _1));
  }

private:

  void topic_callback(const std_msgs::msg::Float32::SharedPtr msg) const
  {
    int Maxi;
    int Mini;
    this->get_parameter("Max",Maxi);
    this->get_parameter("Min",Mini);
    RCLCPP_INFO(this->get_logger(), "I heard: '%f'", msg->data);
    auto percent = ((msg->data)-(Maxi-Mini))*10;
      
    auto message = std_msgs::msg::Float32();
    message.data = percent;
    RCLCPP_INFO(this->get_logger(), "Publishing: '%f'", message.data);
    publisher_->publish(message);
  }
  

  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr publisher_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr subscription_;
  size_t count_;
};


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalPublisher>());
  rclcpp::shutdown();
  return 0;
}
