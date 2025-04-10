#include "formation_controller/gps_heading_to_tf.hpp"
#include "formation_controller/utils.hpp"

GPSHeadingToTF::GPSHeadingToTF() : Node("gps_heading_to_tf")
{
    // Declare parameters
    string current_gps_topic = this->declare_parameter<string>("current_gps_topic", "core/gps");
    string origin_gps_topic = this->declare_parameter<string>("origin_gps_topic", "origin_gps");
    string heading_topic = this->declare_parameter<string>("heading_topic", "/heading");
    parent_frame_ = this->declare_parameter<string>("parent_frame", "world");
    child_frame_ = this->declare_parameter<string>("child_frame", "body");

    // Correct the frame names with namespace
    string ns = this->get_namespace();
    parent_frame_ = replace_ns_prefix(ns, parent_frame_);
    child_frame_ = replace_ns_prefix(ns, child_frame_);

    // Initialize subscribers with lambda functions
    gps_sub_ = this->create_subscription<NavSatFix>(
        current_gps_topic, 1, [this](const NavSatFix::SharedPtr msg) 
        {
            current_gps_ = *msg;
            current_gps_received_ = true;
        });

    origin_gps_sub_ = this->create_subscription<NavSatFix>(
        origin_gps_topic, 1, [this](const NavSatFix::SharedPtr msg) 
        {
            origin_gps_ = *msg;
            origin_gps_received_ = true;
        });

    heading_sub_ = this->create_subscription<Float32>(
        heading_topic, 1, [this](const Float32::SharedPtr msg) 
        {
            current_heading_ = msg->data;
            heading_received_ = true;
        });

    // Initialize transform broadcaster
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

    // Create a timer to publish the transform at 10 Hz
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(100),
        std::bind(&GPSHeadingToTF::timer_callback, this));
}

void GPSHeadingToTF::timer_callback()
{
    if (!current_gps_received_ || !heading_received_) return;
    if (!origin_gps_received_) 
    {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "Waiting for origin GPS data...");
        return;
    }

    // Calculate local position in the body frame
    Point local_position = gps_to_body(current_gps_, origin_gps_);

    // Create a TransformStamped message
    TransformStamped transform;
    transform.header.stamp = this->get_clock()->now();
    transform.header.frame_id = parent_frame_;
    transform.child_frame_id = child_frame_;

    // Set translation
    transform.transform.translation.x = local_position.x;
    transform.transform.translation.y = local_position.y;
    transform.transform.translation.z = local_position.z;

    // Set rotation (convert heading from NED degrees to FLU radians)
    double heading_rad = - current_heading_ * M_PI / 180.0;
    transform.transform.rotation.x = 0.0;
    transform.transform.rotation.y = 0.0;
    transform.transform.rotation.z = sin(heading_rad / 2.0);
    transform.transform.rotation.w = cos(heading_rad / 2.0);

    // Broadcast the transform
    tf_broadcaster_->sendTransform(transform);

    // Reset flags
    current_gps_received_ = false;
    heading_received_ = false;
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<GPSHeadingToTF>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}