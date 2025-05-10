#include "formation_controller/tf_repub.hpp"

RelativeTFListener::RelativeTFListener() : Node("relative_tf_listener"),
  tf_buffer_(this->get_clock()),
  tf_listener_(tf_buffer_)
{
  // Declare and get parameters
  parent_frame_ = this->declare_parameter<string>("parent_frame", "map");
  child_frame_ = this->declare_parameter<string>("child_frame", "");
  string topic_x = this->declare_parameter<string>("topic_x", "");
  string topic_y = this->declare_parameter<string>("topic_y", "");
  string topic_z = this->declare_parameter<string>("topic_z", "");

  // Correct the frame names with namespace
  string ns = this->get_namespace();
  parent_frame_ = replace_ns_prefix(ns, parent_frame_);
  child_frame_ = replace_ns_prefix(ns, child_frame_);

  // Create publishers
  pub_x_ = this->create_publisher<Float32>(topic_x, 10);
  pub_y_ = this->create_publisher<Float32>(topic_y, 10);
  pub_z_ = this->create_publisher<Float32>(topic_z, 10);

  // Create a timer to periodically check for the transform
  timer_ = rclcpp::create_timer(
    this, this->get_clock(), std::chrono::milliseconds(500),
    std::bind(&RelativeTFListener::timer_callback, this));
}

void RelativeTFListener::timer_callback()
{
  TransformStamped transform_stamped;

  try {
    transform_stamped = tf_buffer_.lookupTransform(parent_frame_, child_frame_, tf2::TimePointZero);

    // Extract position data
    float x = transform_stamped.transform.translation.x;
    float y = transform_stamped.transform.translation.y;
    float z = transform_stamped.transform.translation.z;

    // Update previous position data
    if (abs(x - prev_x_) < 1e-6 && abs(y - prev_y_) < 1e-6 && abs(z - prev_z_) < 1e-6) {
      return; // No change in position
    }
    prev_x_ = x; prev_y_ = y; prev_z_ = z;

    // Publish position data
    pub_x_->publish(Float32().set__data(x));
    pub_y_->publish(Float32().set__data(y));
    pub_z_->publish(Float32().set__data(z));
    
  } catch (tf2::TransformException &ex) { }
  
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RelativeTFListener>());
  rclcpp::shutdown();
  return 0;
}