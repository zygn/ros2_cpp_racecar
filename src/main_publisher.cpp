#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <ctime>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "tf2_msgs/msg/tf_message.hpp"

using namespace std::chrono_literals;

// double max_time = 0.0;

// void finally() {
//   std::cout << "max duration: " << max_time << "ms" << std::endl; 
// }

class Racecar : public rclcpp::Node
{
  public:

    Racecar(): Node("racecar")
    {
      this->declare_parameter("bubble_radius", rclcpp::PARAMETER_INTEGER);
      int br_ = this->get_parameter("bubble_radius").as_int();
      this->declare_parameter("preprocess_conv_size", rclcpp::PARAMETER_INTEGER);
      int pcs_ = this->get_parameter("preprocess_conv_size").as_int();
      this->declare_parameter("best_point_conv_size", rclcpp::PARAMETER_INTEGER);
      int bpcs_ = this->get_parameter("best_point_conv_size").as_int();
      this->declare_parameter("maximum_lidar_distance", rclcpp::PARAMETER_DOUBLE);
      double mld_ = this->get_parameter("maximum_lidar_distance").as_double();
      this->declare_parameter("straight_speed", rclcpp::PARAMETER_DOUBLE);
      double ss_ = this->get_parameter("straight_speed").as_double();
      this->declare_parameter("corners_speed", rclcpp::PARAMETER_DOUBLE);
      double cs_ = this->get_parameter("corners_speed").as_double();
      this->declare_parameter("robot_scale", rclcpp::PARAMETER_DOUBLE);
      double rs_ = this->get_parameter("robot_scale").as_double();

      Racecar::setup_parameter(br_, pcs_, bpcs_, mld_, ss_, cs_, rs_);

      this->declare_parameter("scan_topic", "/scan");
      std::string scan_topic_ = this->get_parameter("scan_topic").as_string();
      this->declare_parameter("odom_topic", "/odom");
      std::string odom_topic_ = this->get_parameter("odom_topic").as_string();
      this->declare_parameter("drive_topic", "/drive");
      std::string drive_topic_ = this->get_parameter("drive_topic").as_string();
      this->declare_parameter("tf_topic", "/tf_static");
      std::string tf_topic_ = this->get_parameter("tf_topic").as_string();
      this->declare_parameter("joy_topic", "/joy");
      std::string joy_topic_ = this->get_parameter("joy_topic").as_string(); 

      publisher_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(drive_topic_, 10);
      
      timer_ = this->create_wall_timer(
        10ms, std::bind(&Racecar::timer_callback, this));

      scan_subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        scan_topic_, rclcpp::SensorDataQoS(), std::bind(&Racecar::scan_callback, this, std::placeholders::_1));

      odom_subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
        odom_topic_, rclcpp::SensorDataQoS(), std::bind(&Racecar::odom_callback, this, std::placeholders::_1));

      joy_subscription_ = this->create_subscription<sensor_msgs::msg::Joy>(
        joy_topic_, rclcpp::SensorDataQoS(), std::bind(&Racecar::joy_callback, this, std::placeholders::_1));

      tf_subscription_ = this->create_subscription<tf2_msgs::msg::TFMessage>(
        tf_topic_, rclcpp::SensorDataQoS(), std::bind(&Racecar::tf_callback, this, std::placeholders::_1));

    }


  private:

    sensor_msgs::msg::LaserScan scan_data;
    nav_msgs::msg::Odometry odom_data;
    tf2_msgs::msg::TFMessage tf_data;

  
    const double PI = 3.14159265358979323846;
    const double STRAIGHTS_STEERING_ANGLE = PI / 18;
    int BUBBLE_RADIUS;
    int PREPROCESS_CONV_SIZE;
    int BEST_POINT_CONV_SIZE;
    double MAX_LIDAR_DIST;
    double STRAIGHTS_SPEED;
    double CORNERS_SPEED;
    double radians_per_elem = 0.0;
    double robot_scale;
    int safety = 0;


    void setup_parameter(int br, int pcs, int bpcs, double mld, double ss, double cs, double rs) {
      BUBBLE_RADIUS = br;
      PREPROCESS_CONV_SIZE = pcs;
      BEST_POINT_CONV_SIZE = bpcs;
      MAX_LIDAR_DIST = mld;
      STRAIGHTS_SPEED = ss;
      CORNERS_SPEED = cs;
      robot_scale = rs;
      return;
    }

    void joy_callback(sensor_msgs::msg::Joy::SharedPtr msg) {
      safety = msg->buttons[7];
    }

    void timer_callback()
    { 
      if (scan_data.ranges.size() == 0) {
        RCLCPP_WARN(this->get_logger(), "scan data size is '%d' skipping this callback.", scan_data.ranges.size());
        return;
      }

      if (safety == 0) {
        // Topic sensor_msg/msg/Joy->buttons[7] is RB Button 
        RCLCPP_WARN_ONCE(this->get_logger(), "Skipping this callback.. Press joy [RB] button");
        auto message = ackermann_msgs::msg::AckermannDriveStamped();
        message.drive.speed = 0.0;
        message.drive.steering_angle = 0.0;
        publisher_->publish(message);
        return;
      }

      clock_t start, end;
      // auto const count = static_cast<float>(scan_data.ranges.size());
      // RCLCPP_INFO(this->get_logger(), "scan data avg value: '%f'", std::reduce(scan_data.ranges.begin(), scan_data.ranges.end()) / count);
      
      auto message = ackermann_msgs::msg::AckermannDriveStamped();
      std::vector<float> ranges = scan_data.ranges;

      start = clock();
      auto results = driving(ranges);
      // connect driving fuction 
      // results.first : speed, results.second : angle
      message.drive.speed = results.first;
      message.drive.steering_angle = results.second;

      publisher_->publish(message);
      end = clock();
      auto duration = (double)(end - start) / (CLOCKS_PER_SEC/1000);

      // if (max_time < duration) {
      //   max_time = duration;
      // }
      RCLCPP_INFO(this->get_logger(), "speed: '%.4f', steer: '%.4f', duration: '%fms'", results.first, results.second, duration);
    }



    void scan_callback(sensor_msgs::msg::LaserScan::SharedPtr msg) 
    {
      scan_data.angle_increment = msg->angle_increment;
      scan_data.angle_max = msg->angle_max;
      scan_data.angle_min = msg->angle_min;
      
      std::vector<float> pol = msg->ranges;

      /// remove this block when your lidar is running okay...
      for (int i=0; i>pol.size(); i++) {
        if (i>5 && pol[i] == 0.0 && (i<pol.size()-5)) {
          std::vector<float> blk;
          for (int j=i-5; j<i+5; j++) {
            if (pol[j] != 0.0) {
              blk.push_back(pol[j]);
            }
          }
          pol[i] = blk.empty() ? 0.2 : std::accumulate(blk.begin(), blk.end(), 0.0) / blk.size();
        }
      }
      ///

      scan_data.ranges = pol;
      scan_data.range_max = msg->range_max;
      scan_data.range_min = msg->range_min;
      
      scan_data.header = msg->header;
    }

    void odom_callback(nav_msgs::msg::Odometry::SharedPtr msg) 
    {      
      odom_data.pose = msg->pose;
      odom_data.twist = msg->twist;
      odom_data.header = msg->header;
      odom_data.child_frame_id = msg->child_frame_id;
    }

    void tf_callback(tf2_msgs::msg::TFMessage::SharedPtr msg)
    {
      tf_data.transforms = msg->transforms;
    }

    /*
    
    Driving Functions..

    */

    std::vector<double> preprocessLidar(const std::vector<double>& ranges) {
        radians_per_elem = (3.0 / 2.0 * PI) / ranges.size();
        std::vector<double> proc_ranges(ranges.size());

        for (size_t i = 0; i < ranges.size(); ++i) {
            proc_ranges[i] = std::min(std::max(ranges[i], 0.0), MAX_LIDAR_DIST);
        }

        std::vector<double> conv_kernel(PREPROCESS_CONV_SIZE, 1.0 / PREPROCESS_CONV_SIZE);
        std::vector<double> conv_result(proc_ranges.size());

        for (size_t i = 0; i < proc_ranges.size(); ++i) {
            double sum = 0.0;
            for (size_t j = 0; j < conv_kernel.size(); ++j) {
                int range_index = static_cast<int>(i) - static_cast<int>(conv_kernel.size()) / 2 + static_cast<int>(j);
                if (range_index >= 0 && range_index < static_cast<int>(proc_ranges.size())) {
                    sum += proc_ranges[range_index] * conv_kernel[j];
                }
            }
            conv_result[i] = sum;
        }

        return conv_result;
    }

    std::pair<int, int> findMaxGap(const std::vector<double>& free_space_ranges) {
        std::vector<double> masked(free_space_ranges.size());
        for (int i = 0; i < free_space_ranges.size(); ++i) {
            masked[i] = (free_space_ranges[i] == 0) ? std::numeric_limits<double>::quiet_NaN() : free_space_ranges[i];
        }

        std::vector<std::pair<int, int>> slices;
        auto it = masked.begin();
        while (it != masked.end()) {
            it = std::find_if_not(it, masked.end(), [](double val) {
                return std::isnan(val);
            });
            if (it != masked.end()) {
                auto start = std::distance(masked.begin(), it);
                it = std::find_if(it, masked.end(), [](double val) {
                    return std::isnan(val);
                });
                auto stop = distance(masked.begin(), it);
                slices.push_back(std::make_pair(start, stop));
            }
        }

        if (slices.empty()) {
            return std::make_pair(0, 0);
        }

        auto cmpLength = [](const std::pair<int, int>& slice1, const std::pair<int, int>& slice2) {
            return (slice1.second - slice1.first) < (slice2.second - slice2.first);
        };

        auto maxSlice = *std::max_element(slices.begin(), slices.end(), cmpLength);

        return maxSlice;
    }

    int findBestPoint(int start_i, int end_i, const std::vector<double>& ranges) {
        std::vector<double> subRanges(ranges.begin() + start_i, ranges.begin() + end_i);
        std::vector<double> averagedMaxGap(subRanges.size());

        for (int i = 0; i < subRanges.size(); ++i) {
            double sum = 0.0;
            for (int j = std::max(0, i - BEST_POINT_CONV_SIZE / 2); j <= std::min(static_cast<int>(subRanges.size()) - 1, i + BEST_POINT_CONV_SIZE / 2); ++j) {
                sum += subRanges[j];
            }
            averagedMaxGap[i] = sum / BEST_POINT_CONV_SIZE;
        }

        auto maxElementIt = std::max_element(averagedMaxGap.begin(), averagedMaxGap.end());
        int maxIndex = std::distance(averagedMaxGap.begin(), maxElementIt);

        return maxIndex + start_i;
    }

    double getAngle(int range_index, int range_len) {
        double lidar_angle = (range_index - (range_len / 2)) * radians_per_elem;
        double steering_angle = lidar_angle / 2.0;

        return steering_angle;
    }

    std::pair<float, float> driving(const std::vector<float> &scan_data) {
      // tpyecast float to double
      std::vector<double> ranges(std::begin(scan_data), std::end(scan_data));
      std::vector<double> proc_ranges = preprocessLidar(ranges);
      auto minElementIt = std::min_element(proc_ranges.begin(), proc_ranges.end());

      int closest = std::distance(proc_ranges.begin(), minElementIt);

      int min_index = closest - BUBBLE_RADIUS;
      int max_index = closest + BUBBLE_RADIUS;
      if (min_index < 0) min_index = 0;
      if (max_index >= proc_ranges.size()) max_index = proc_ranges.size() - 1;
      std::fill(proc_ranges.begin() + min_index, proc_ranges.begin() + max_index, 0.0);

      auto gaps = findMaxGap(proc_ranges);

      int best = findBestPoint(gaps.first, gaps.second, proc_ranges);

      double steering_angle = getAngle(best, proc_ranges.size());
      double speed = (abs(steering_angle) > STRAIGHTS_STEERING_ANGLE) ? CORNERS_SPEED : STRAIGHTS_SPEED;

      // std::cout << "Steering angle in degrees: " << (steering_angle / (PI / 2)) * 90 << std::endl;
      // std::cout << "Speed: " << speed << std::endl;

      return std::make_pair(speed, steering_angle);

      
    }

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr publisher_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscription_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_;
    rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr tf_subscription_;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_subscription_;


};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Racecar>() );
  rclcpp::shutdown();
  //finally();
 
  return 0;
}
