#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "std_msgs/msg/string.hpp"

class DigitalTwinBridge : public rclcpp::Node
{
public:
    DigitalTwinBridge() : Node("digital_twin_bridge")
    {
        RCLCPP_INFO(this->get_logger(), "Initializing Digital Twin Bridge Node");

        // Create publishers for sensor data to Unity
        joint_state_pub_ = this->create_publisher<sensor_msgs::msg::JointState>(
            "/unity/joint_states", 10);
        scan_pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>(
            "/unity/scan", 10);
        image_pub_ = this->create_publisher<sensor_msgs::msg::Image>(
            "/unity/camera/image_raw", 10);
        imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>(
            "/unity/imu/data", 10);
        odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>(
            "/unity/odom", 10);

        // Create subscribers for commands from Unity
        cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/unity/cmd_vel", 10,
            std::bind(&DigitalTwinBridge::cmdVelCallback, this, std::placeholders::_1));

        // Create timer for synchronization
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(50),  // 20 Hz
            std::bind(&DigitalTwinBridge::timerCallback, this));

        RCLCPP_INFO(this->get_logger(), "Digital Twin Bridge Node initialized");
    }

private:
    // Publishers
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;

    // Subscribers
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;

    // Timer
    rclcpp::TimerBase::SharedPtr timer_;

    // Callbacks
    void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "Received cmd_vel from Unity: linear=%.2f, angular=%.2f",
            msg->linear.x, msg->angular.z);

        // Forward command to Gazebo
        auto cmd_pub = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        cmd_pub->publish(*msg);
    }

    void timerCallback()
    {
        // Synchronize data between Gazebo and Unity
        syncJointStates();
        syncSensorData();
    }

    void syncJointStates()
    {
        // Create and publish joint state message
        auto joint_msg = sensor_msgs::msg::JointState();
        joint_msg.header.stamp = this->get_clock()->now();
        joint_msg.header.frame_id = "base_link";

        // In a real implementation, these would come from Gazebo
        joint_msg.name = {"left_arm_joint", "right_arm_joint", "left_leg_joint", "right_leg_joint"};
        joint_msg.position = {0.0, 0.0, 0.0, 0.0}; // Placeholder values
        joint_msg.velocity = {0.0, 0.0, 0.0, 0.0};
        joint_msg.effort = {0.0, 0.0, 0.0, 0.0};

        joint_state_pub_->publish(joint_msg);
    }

    void syncSensorData()
    {
        // Publish dummy sensor data (in real implementation, read from Gazebo)
        auto scan_msg = sensor_msgs::msg::LaserScan();
        scan_msg.header.stamp = this->get_clock()->now();
        scan_msg.header.frame_id = "lidar_link";
        scan_msg.angle_min = -3.14159;
        scan_msg.angle_max = 3.14159;
        scan_msg.angle_increment = 0.01745; // 1 degree
        scan_msg.time_increment = 0.0;
        scan_msg.scan_time = 0.1;
        scan_msg.range_min = 0.1;
        scan_msg.range_max = 10.0;

        // Fill with dummy data
        scan_msg.ranges.resize(360, 5.0); // 360 readings at 5m
        scan_msg.intensities.resize(360, 1.0);

        scan_pub_->publish(scan_msg);
    }
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DigitalTwinBridge>());
    rclcpp::shutdown();
    return 0;
}