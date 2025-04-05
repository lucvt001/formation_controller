#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <std_msgs/msg/float32.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>

using namespace std;
using Float32 = std_msgs::msg::Float32;
using NavSatFix = sensor_msgs::msg::NavSatFix;
using TransformStamped = geometry_msgs::msg::TransformStamped;

class OriginPub : public rclcpp::Node
{
public:
  OriginPub() : Node("origin_pub")
  {
    // Declare and get parameters
    string origin_gps_input_topic = this->declare_parameter<string>("origin_gps_input_topic", "core/gps");
    string origin_gps_output_topic = this->declare_parameter<string>("origin_gps_output_topic", "origin_gps");
    string heading_topic = this->declare_parameter<string>("heading_topic", "core/heading");
    string world_frame = this->declare_parameter<string>("world_frame", "world");
    string local_frame = this->declare_parameter<string>("local_frame", "map");

    // Initialize subscriber
    gps_sub_ = this->create_subscription<NavSatFix>(
      origin_gps_input_topic, 10,
      [this](const NavSatFix::SharedPtr msg) {
        if (origin_gps_received_) return;
        origin_gps_ = *msg;
        origin_gps_received_ = true;
        RCLCPP_INFO(this->get_logger(), "Origin GPS received: [lat: %f, lon: %f, alt: %f]",
          origin_gps_.latitude, origin_gps_.longitude, origin_gps_.altitude);
      });

    heading_sub_ = this->create_subscription<Float32>(
      heading_topic, 10,
      [this](const Float32::SharedPtr msg) {
        if (heading_received_) return;
        float heading = msg->data;
        float heading_rad = - heading * M_PI / 180.0; // Convert degrees to radians in FLU
        heading_received_ = true;
        RCLCPP_INFO(this->get_logger(), "Heading received: %f", heading);
        // Create transform
        tf2::Quaternion q;
        q.setRPY(0, 0, heading_rad); q.normalize();
        transform_msg_.transform.rotation.x = q.x();
        transform_msg_.transform.rotation.y = q.y();
        transform_msg_.transform.rotation.z = q.z();
        transform_msg_.transform.rotation.w = q.w();
      });

    // Initialize publisher
    gps_pub_ = this->create_publisher<NavSatFix>(origin_gps_output_topic, 10);

    // Initialize transform broadcaster
    tf_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);

    // Set up the transform message
    transform_msg_.header.frame_id = world_frame;
    transform_msg_.child_frame_id = local_frame;

    // Initialize timer to publish the origin GPS periodically
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(1000),
      std::bind(&OriginPub::timer_callback, this));
  }

private:
  void timer_callback()
  {
    if (origin_gps_received_) {
      gps_pub_->publish(origin_gps_);
    } else {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "Waiting for origin GPS...");
    }

    if (heading_received_) {
      // Publish the transform
      transform_msg_.header.stamp = this->now();
      tf_broadcaster_->sendTransform(transform_msg_);
    } else {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "Waiting for heading...");
    }
  }

  // Subscriber and publisher
  rclcpp::Subscription<NavSatFix>::SharedPtr gps_sub_;
  rclcpp::Publisher<NavSatFix>::SharedPtr gps_pub_;
  rclcpp::Subscription<Float32>::SharedPtr heading_sub_;

  // Transform broadcaster
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_broadcaster_;
  string parent_frame_;
  string child_frame_;
  TransformStamped transform_msg_;

  // Data
  NavSatFix origin_gps_;
  Float32 heading_;
  bool origin_gps_received_ = false;
  bool heading_received_ = false;

  // Timer
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<OriginPub>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}