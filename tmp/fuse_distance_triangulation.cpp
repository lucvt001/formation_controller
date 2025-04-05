#include "formation_controller/fuse_distance_triangulation.hpp"

FuseDistanceTriangulation::FuseDistanceTriangulation()
    : Node("fuse_distance_triangulation"), y_(0.0)
{
    string leader1_distance_topic = this->declare_parameter<string>("leader1_distance_topic", "");
    string leader2_distance_topic = this->declare_parameter<string>("leader2_distance_topic", "");
    string follower_position_topic = this->declare_parameter<string>("follower_relative_position_triangulation_topic", "");
    string follower_depth_topic = this->declare_parameter<string>("follower_depth_topic", "");

    follower_depth_offset_ = this->declare_parameter<float>("follower_depth_offset", 0.0);
    leaders_distance_ = this->declare_parameter<float>("leaders_distance", 0.0);

    leader1_sub_ = this->create_subscription<Float32>(
        leader1_distance_topic, 10, [this](const Float32::SharedPtr msg) {
            leader1_distance_ = msg->data;
            triangulatePosition();
        });

    leader2_sub_ = this->create_subscription<Float32>(
        leader2_distance_topic, 10, [this](const Float32::SharedPtr msg) {
            leader2_distance_ = msg->data;
            triangulatePosition();
        });

    follower_depth_sub_ = this->create_subscription<FluidPressure>(
        follower_depth_topic, 10, [this](const FluidPressure::SharedPtr msg) {
            float pressure_depth = msg->fluid_pressure / 9806.65; // Convert pressure to depth in meters
            follower_depth_ = pressure_depth - follower_depth_offset_;
        });

    follower_position_pub_ = this->create_publisher<Point>(follower_position_topic, 10);

}

void FuseDistanceTriangulation::triangulatePosition()
{
    if (leader1_distance_ < 0 || leader2_distance_ < 0) return;

    // Assuming leader2 is at (d, 0) relative to leader1 at (0, 0)
    float d = leaders_distance_; // Distance between leader1 and leader2 (example value)
    float d1 = sqrt(pow(leader1_distance_, 2) - pow(follower_depth_, 2));
    float d2 = sqrt(pow(leader2_distance_, 2) - pow(follower_depth_, 2));

    // Imagine a triangle with three sides: d1, d2, and d
    // Using the Law of Cosines to find the angle between d and d1
    float cosine = (pow(d1, 2) + pow(d, 2) - pow(d2, 2)) / (2 * d1 * d);
    float angle = acos(cosine);
    float sine = sin(angle);

    Point follower_position;
    follower_position.x = - d1 * sine;
    // if (abs(y_) < 0.0001) y_ = d1 * cosine;
    // else y_ = y_ * 0.6 + d1 * cosine * 0.4;
    // follower_position.y = y_;
    follower_position.y = d1 * cosine;
    follower_position.z = follower_depth_;
    // cout << "follower position: (" << follower_position.x << ", " << follower_position.y << ", " << follower_position.z << ")" << endl;

    follower_position_pub_->publish(follower_position);
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FuseDistanceTriangulation>());
    rclcpp::shutdown();
    return 0;
}