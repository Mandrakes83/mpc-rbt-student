#include <cstdio>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "sensor_msgs/msg/range.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float32.hpp"

using namespace std::chrono_literals;

class TestNode : public rclcpp::Node {
public:
    TestNode() : Node("ros2_lab_test_node") {
        RCLCPP_INFO(get_logger(), "\n\nWelcome to the MPC-RBT ROS2 lab!\nThis node has been created to test your understanding of ROS2 framework and your ability to use it.\nFollow the lab GIT readme step by step - don't skip!\nUse this console output to check whether you have successfully completed each step.\n\nProceed with the step 1.\n");

        subscription0_ = create_subscription<std_msgs::msg::Bool>("student_ready", 1, std::bind(&TestNode::topic_0_callback, this, std::placeholders::_1));

        subscription1_ = create_subscription<sensor_msgs::msg::Range>("range_sensor", 1, std::bind(&TestNode::topic_1_callback, this, std::placeholders::_1));

        robotPoseX = static_cast<int8_t>(rand());
        robotPoseY = static_cast<int8_t>(rand());
        publisher2_ = this->create_publisher<geometry_msgs::msg::Point>("robot_position", 1);
        timer2_ = this->create_wall_timer(50ms, std::bind(&TestNode::timer_2_callback, this));
        subscription2_ = create_subscription<geometry_msgs::msg::Point>("robot_move", 1, std::bind(&TestNode::topic_2_callback, this, std::placeholders::_1));

        subscription3_ = create_subscription<std_msgs::msg::String>("node_name", 1, std::bind(&TestNode::topic_3_callback, this, std::placeholders::_1));

        auto param_desc = rcl_interfaces::msg::ParameterDescriptor{};
        param_desc.description = "This parameter changes the publishing frequency of the battery voltage masurement!";
        this->declare_parameter("battery_voltage_hz", 2, param_desc);

        createTimer4();
        publisher4_ = this->create_publisher<std_msgs::msg::Float32>("battery_voltage", 1);
        subscription4_ = create_subscription<std_msgs::msg::Float32>("battery_percentage", 1, std::bind(&TestNode::topic_4_callback, this, std::placeholders::_1));

        paramSubscriber_ = std::make_shared<rclcpp::ParameterEventHandler>(this);

        auto cb = [this](const rclcpp::Parameter & p) {
            if (currentStep_ != 5) { return; }

            RCLCPP_INFO(
              this->get_logger(), "Received an update to parameter \"%s\" of type %s: \"%ld\"",
              p.get_name().c_str(),
              p.get_type_name().c_str(),
              p.as_int());

            createTimer4();
            RCLCPP_INFO(this->get_logger(), "Congrats, step 6 finished!");
            currentStep_ = 6;
        };
        cbHandle_ = paramSubscriber_->add_parameter_callback("battery_voltage_hz", cb);
    }

private:
    void topic_0_callback(const std_msgs::msg::Bool::ConstSharedPtr &msg) {
        if (currentStep_ != 0) { return; }
        if (msg->data) {
            currentStep_ = 1;
            RCLCPP_INFO(this->get_logger(), "Glad you're ready - Step 1 completed!\n");
        }
    }

    void topic_1_callback(const sensor_msgs::msg::Range::ConstSharedPtr &msg) {
        static uint8_t msgsReceived = 0;

        if (currentStep_ == 2) { // Student sent too many messages, back to step 1
            currentStep_ = 1;
            msgsReceived = 0;
            RCLCPP_INFO(this->get_logger(), "Oops, too many, try again!\n");
        }
        if (currentStep_ != 1) { return; }

        if (msg->min_range == 1 && msg->max_range == 10 && msg->range >= 1 && msg->range <= 10 && msg->header.frame_id == "range_sensor_front") {
            msgsReceived++;
            if (msgsReceived == 100) {
                currentStep_ = 2;
                RCLCPP_INFO(this->get_logger(), "Great - Step 2 completed!\n");
            }

        }
    }

    void timer_2_callback() {
        if (currentStep_ != 2) { return; }
        auto msg = geometry_msgs::msg::Point();
        msg.x = robotPoseX;
        msg.y = robotPoseY;
        msg.z = robotPoseZ;
        publisher2_->publish(msg);

        if (robotPoseX == 0 && robotPoseY == 0 && robotPoseZ == 0) {
            currentStep_ = 3;
            RCLCPP_INFO(this->get_logger(), "Robot is home, thanks - Step 3 completed!\n\n");
            RCLCPP_INFO(this->get_logger(), "Now is the right time to create your own package. Proceed with step 4\n");
        }
    }

    void topic_2_callback(const geometry_msgs::msg::Point::ConstSharedPtr &msg) {
        if (currentStep_ != 2) { return; }

        auto moveX = msg->x;
        if (moveX > 10) moveX = 10;
        if (moveX < -10) moveX = -10;
        auto moveY = msg->y;
        if (moveY > 10) moveY = 10;
        if (moveY < -10) moveY = -10;
        auto moveZ = msg->z;
        if (moveZ > 10) moveZ = 10;
        if (moveZ < -10) moveZ = -10;

        robotPoseX += moveX;
        robotPoseY += moveY;
        robotPoseZ += moveZ;
    }

    void topic_3_callback(const std_msgs::msg::String::ConstSharedPtr &msg) {
        if (currentStep_ != 3) {  return; }

        auto nodeName = this->get_publishers_info_by_topic("node_name").at(0).node_name();
        if (msg->data == nodeName) {
            currentStep_ = 4;
            RCLCPP_INFO(this->get_logger(), "Congratulations to creating your first C++ node! Continue with step 5\n");
        } else {
            RCLCPP_INFO(this->get_logger(), "Node name and topic string are not equal!\n");
        }

    }

    void timer_4_callback() {
        if (currentStep_ != 4 && currentStep_ != 5) { return; }
        batteryVoltage = 32.0f + 10.0f  *(static_cast<float>(rand()) / RAND_MAX);
        batteryPercentage = ((batteryVoltage - 32.0f) / 10.0f) * 100.0f;

        auto msg = std_msgs::msg::Float32();
        msg.data = batteryVoltage;
        publisher4_->publish(msg);
    }

    void topic_4_callback(const std_msgs::msg::Float32::ConstSharedPtr &msg) {
        if (currentStep_ != 4) { return; }

        static int timesReceived = 0;

        auto diff = abs(msg->data - batteryPercentage);
        if (diff < 0.01) {
            timesReceived++;
            RCLCPP_INFO(this->get_logger(), ".");
            if (timesReceived >= 10) {
                currentStep_ = 5;
                RCLCPP_INFO(this->get_logger(), "Your node is now correctly transforming battery voltage! Continue with step 6\n");
            }
        } else {
            RCLCPP_WARN(this->get_logger(), "Incorrect battery percentage by %f", diff);
        }
    }

    void createTimer4() {
        auto freq = this->get_parameter("battery_voltage_hz").as_int();
        auto period = std::chrono::duration<double>(1.0 / static_cast<float>(freq));

        timer4_ = this->create_wall_timer(std::chrono::duration_cast<std::chrono::milliseconds>(period), std::bind(&TestNode::timer_4_callback, this));
    }

    uint8_t currentStep_ = 3;

    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr subscription0_;
    rclcpp::Subscription<sensor_msgs::msg::Range>::SharedPtr subscription1_;

    int8_t robotPoseX, robotPoseY, robotPoseZ;
    rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr publisher2_;
    rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr subscription2_;
    rclcpp::TimerBase::SharedPtr timer2_;

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription3_;

    float batteryVoltage, batteryPercentage;
    rclcpp::TimerBase::SharedPtr timer4_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr publisher4_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr subscription4_;

    std::shared_ptr<rclcpp::ParameterEventHandler> paramSubscriber_;
    std::shared_ptr<rclcpp::ParameterCallbackHandle> cbHandle_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TestNode>());
    rclcpp::shutdown();
    return 0;
}
