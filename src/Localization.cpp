#include "mpc_rbt_simulator/RobotConfig.hpp"
#include "Localization.hpp"
#include <sstream>
#include <vector>

LocalizationNode::LocalizationNode() : 
    rclcpp::Node("localization_node"), 
    last_time_(this->get_clock()->now()) {

    // Odometry message initialization
    odometry_.pose.pose.position.x = -0.5;
    odometry_.pose.pose.position.y = 0;
    odometry_.pose.pose.position.z = 0;
    odometry_.pose.pose.orientation.x = 0;
    odometry_.pose.pose.orientation.y = 0;
    odometry_.pose.pose.orientation.z = 0;
    odometry_.pose.pose.orientation.w = 1;
    odometry_.header.frame_id = "map";
    odometry_.child_frame_id = "base_link";
    //add code here

    // Subscriber for joint_states
    joint_subscriber_ = this->create_subscription<sensor_msgs::msg::JointState>("/joint_states",1,std::bind(&LocalizationNode::jointCallback,this,std::placeholders::_1));
    current_time_ = this->get_clock()->now();

    // Publisher for odometry
    odometry_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("odometry",1);

    // tf_briadcaster 
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    RCLCPP_INFO(get_logger(), "Localization node started.");
}

void LocalizationNode::jointCallback(const sensor_msgs::msg::JointState & msg) {
    // add code here
    std::ostringstream oss;
    oss << "Position: [";
    for (size_t i = 0; i < msg.position.size(); ++i) {
      oss << msg.position[i];
      if (i != msg.position.size() - 1) {
        oss << ", ";
      }
    }
    oss << "]";
    //RCLCPP_INFO(this->get_logger(), "%s", oss.str().c_str());

    last_time_ = current_time_;
    current_time_ = this->get_clock()->now();
    double dt = (current_time_.nanoseconds() - last_time_.nanoseconds())/1000000000.0;
    updateOdometry(msg.velocity[0], msg.velocity[1], dt);
    publishOdometry();
    publishTransform();
    
}

void LocalizationNode::updateOdometry(double left_wheel_vel, double right_wheel_vel, double dt) {
    // add code here

    // ********
    // * Help *
    // ********
    
    double linear =  (left_wheel_vel*robot_config::WHEEL_RADIUS + right_wheel_vel*robot_config::WHEEL_RADIUS)/2.0;
    double angular = -(right_wheel_vel*robot_config::WHEEL_RADIUS-left_wheel_vel*robot_config::WHEEL_RADIUS)/(robot_config::HALF_DISTANCE_BETWEEN_WHEELS*2.0);  

    
    tf2::Quaternion tf_quat;
    tf2::fromMsg(odometry_.pose.pose.orientation, tf_quat);
    double roll, pitch, theta;
    tf2::Matrix3x3(tf_quat).getRPY(roll, pitch, theta);

    theta = std::atan2(std::sin(theta), std::cos(theta));

    odometry_.pose.pose.position.x += linear * dt * std::cos(theta);
    odometry_.pose.pose.position.y += linear * dt * std::sin(theta);
    theta += angular * dt;

    tf2::Quaternion q;
    q.setRPY(0, 0, theta);
    odometry_.pose.pose.orientation = tf2::toMsg(q);
    
}

void LocalizationNode::publishOdometry() {
    odometry_.header.stamp = this->get_clock()->now();
    odometry_publisher_->publish(odometry_);
}

void LocalizationNode::publishTransform() {
    geometry_msgs::msg::TransformStamped transform;
    transform.header.stamp = this->get_clock()->now();
    transform.header.frame_id = "map";
    transform.child_frame_id = "base_link";

    transform.transform.translation.x = odometry_.pose.pose.position.x;
    transform.transform.translation.y = odometry_.pose.pose.position.y;
    transform.transform.translation.z = 0.0;
    transform.transform.rotation = odometry_.pose.pose.orientation;
    // ********
    // * Help *
    // ********
    tf_broadcaster_->sendTransform(transform);
}
