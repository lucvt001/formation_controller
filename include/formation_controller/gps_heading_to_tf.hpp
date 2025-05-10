#ifndef GPS_HEADING_TO_TF_HPP
#define GPS_HEADING_TO_TF_HPP

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <std_msgs/msg/float32.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include "gps_to_utm.hpp"

using namespace std;
using NavSatFix = sensor_msgs::msg::NavSatFix;
using Float32 = std_msgs::msg::Float32;
using Point = geometry_msgs::msg::Point;
using TransformStamped = geometry_msgs::msg::TransformStamped;

class GPSHeadingToTF : public rclcpp::Node
{
public:
    GPSHeadingToTF();

private:
    void timer_callback();

    // Subscribers
    rclcpp::Subscription<NavSatFix>::SharedPtr gps_sub_, origin_gps_sub_;
    rclcpp::Subscription<Float32>::SharedPtr heading_sub_;

    // Transform broadcaster
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    // Parameters
    string parent_frame_;
    string child_frame_;

    // Data
    NavSatFix origin_gps_, current_gps_;
    double current_heading_; // Heading in degrees
    bool current_gps_received_ = false;
    bool origin_gps_received_ = false;
    bool heading_received_ = false;

    // Timer
    rclcpp::TimerBase::SharedPtr timer_;
};

#endif // GPS_HEADING_TO_TF_HPP