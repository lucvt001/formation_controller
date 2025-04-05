#ifndef RELATIVE_TF_LISTENER_HPP
#define RELATIVE_TF_LISTENER_HPP

#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <std_msgs/msg/float32.hpp>
#include "utils.hpp"

using namespace std;
using TransformStamped = geometry_msgs::msg::TransformStamped;
using Float32 = std_msgs::msg::Float32;

class RelativeTFListener : public rclcpp::Node
{
public:
  RelativeTFListener();

private:
  void timer_callback();

  string parent_frame_;
  string child_frame_;

  rclcpp::Publisher<Float32>::SharedPtr pub_x_;
  rclcpp::Publisher<Float32>::SharedPtr pub_y_;
  rclcpp::Publisher<Float32>::SharedPtr pub_z_;
  float prev_x_, prev_y_, prev_z_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  rclcpp::TimerBase::SharedPtr timer_;
};

#endif // RELATIVE_TF_LISTENER_HPP