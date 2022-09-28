#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

using namespace std::chrono_literals;


class Racecar : public rclcpp::Node
{
  public:

    Racecar(): Node("racecar")
    {
      publisher_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>("/drive", 10);
      timer_ = this->create_wall_timer(
        10ms, std::bind(&Racecar::timer_callback, this));

      scan_subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "scan", 10, std::bind(&Racecar::scan_callback, this, std::placeholders::_1));

      odom_subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "ego_racecar/odom", 10, std::bind(&Racecar::odom_callback, this, std::placeholders::_1));
      
    }
    
  private:

    sensor_msgs::msg::LaserScan scan_data;
    nav_msgs::msg::Odometry odom_data;

    void timer_callback()
    {
      auto message = ackermann_msgs::msg::AckermannDriveStamped();

      // TODO: 
      // speed, steer = driving(scan_data, odom_data)

      // connect driving fuction 
      message.drive.speed = 1.0;
      message.drive.steering_angle = 0.0;

      publisher_->publish(message);
    }

    void scan_callback(sensor_msgs::msg::LaserScan::SharedPtr msg) 
    {
      scan_data.angle_increment = msg->angle_increment;
      scan_data.angle_max = msg->angle_max;
      scan_data.angle_min = msg->angle_min;
      
      scan_data.ranges = msg->ranges;
      scan_data.range_max = msg->range_max;
      scan_data.range_min = msg->range_min;
      
      scan_data.header = msg->header;

      // RCLCPP_INFO(this->get_logger(), "test value: '%f'", scan_data.ranges[540]);
    }

    void odom_callback(nav_msgs::msg::Odometry::SharedPtr msg) 
    {      
      odom_data.pose = msg->pose;
      odom_data.twist = msg->twist;
      odom_data.header = msg->header;
      odom_data.child_frame_id = msg->child_frame_id;
      // RCLCPP_INFO(this->get_logger(), "test value: '%f'", odom_data.pose.pose.position.x);
    }

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr publisher_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscription_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_;

};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Racecar>());
  rclcpp::shutdown();
  return 0;
}