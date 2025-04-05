#ifndef FUSE_DISTANCE_TRIANGULATION_HPP
#define FUSE_DISTANCE_TRIANGULATION_HPP

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <sensor_msgs/msg/fluid_pressure.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <cmath>

using namespace std;
using Float32 = std_msgs::msg::Float32;
using FluidPressure = sensor_msgs::msg::FluidPressure;
using Point = geometry_msgs::msg::Point;

class FuseDistanceTriangulation : public rclcpp::Node
{
public:
    FuseDistanceTriangulation();

private:
    void triangulatePosition();

    rclcpp::Subscription<Float32>::SharedPtr leader1_sub_;
    rclcpp::Subscription<Float32>::SharedPtr leader2_sub_;
    rclcpp::Subscription<FluidPressure>::SharedPtr follower_depth_sub_;
    rclcpp::Publisher<Point>::SharedPtr follower_position_pub_;

    float leader1_distance_ = -1.0;
    float leader2_distance_ = -1.0;
    float follower_depth_;
    float follower_depth_offset_;
    float leaders_distance_;
    float y_;
};

#endif // FUSE_DISTANCE_TRIANGULATION_HPP