#ifndef RELATIVE_TARGET_POSITION_PUBLISHER_HPP
#define RELATIVE_TARGET_POSITION_PUBLISHER_HPP

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <Eigen/Dense>

using namespace std;
using Float32 = std_msgs::msg::Float32;
using Point = geometry_msgs::msg::Point;

class RelativeTargetPositionPublisher : public rclcpp::Node
{
public:
  /* This class will subscribe to a topic containing 3D position of the follower relative to leader 1
  Subtract from this position the target point that we want the follower to be at relative to leader 1
  To obtain the position of the follower relative to the target point
  And publish this data to three separate topics */
  RelativeTargetPositionPublisher();

private:
  void pointCallback(const Point::SharedPtr msg);
  // float calculateAngle(const Eigen::Vector3f& vec1, const Eigen::Vector3f& vec2);

  rclcpp::Subscription<Point>::SharedPtr point_sub_;
  rclcpp::Publisher<Float32>::SharedPtr x_pub_;
  rclcpp::Publisher<Float32>::SharedPtr y_pub_;
  rclcpp::Publisher<Float32>::SharedPtr z_pub_;
  rclcpp::Publisher<Point>::SharedPtr follower_offset_pub_; 
  rclcpp::Publisher<Point>::SharedPtr og_target_pub_;     // Publish the original target point relative to leader 1. Purely for visualization purposes

  float offset_x_;
  float offset_y_;
  float offset_z_;
  Eigen::Vector3f prev_relative_position_;

  bool toggle_;
  float prev_angle_;
};

#endif // RELATIVE_TARGET_POSITION_PUBLISHER_HPP