#ifndef TF_TO_PATH_HPP
#define TF_TO_PATH_HPP

#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <string>

using namespace std;
using TransformStamped = geometry_msgs::msg::TransformStamped;
using PoseStamped = geometry_msgs::msg::PoseStamped;
using Path = nav_msgs::msg::Path;

class TFToPath : public rclcpp::Node
{
public:
  TFToPath();

private:
  void timer_callback();

  // Parameters
  string parent_frame_;
  string child_frame_;

  // TF2
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  // Publisher
  rclcpp::Publisher<Path>::SharedPtr path_pub_;

  // Timer
  rclcpp::TimerBase::SharedPtr timer_;

  // Path message
  Path path_msg_;
};

#endif // TF_TO_PATH_HPP