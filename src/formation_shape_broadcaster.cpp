#include "formation_controller/formation_shape_broadcaster.hpp"
#include <fstream>

FormationShapeBroadcaster::FormationShapeBroadcaster() : Node("formation_shape_broadcaster")
{
  // Declare and get the YAML file path parameter
  string yaml_file_path = this->declare_parameter<string>("yaml_file_path", "formation_shape.yaml");

  // Leader frame name
  leader_frame_ = this->declare_parameter<string>("leader_frame", "supreme_leader");

  // Load the YAML file
  load_yaml(yaml_file_path);

  // Initialize the static transform broadcaster
  static_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);

  // Create a timer to publish transforms at 1 Hz
  timer_ = rclcpp::create_timer(
    this, this->get_clock(), std::chrono::milliseconds(1000),
    std::bind(&FormationShapeBroadcaster::publish_transforms, this));
}

void FormationShapeBroadcaster::load_yaml(const string &file_path)
{
  try
  {
    YAML::Node config = YAML::LoadFile(file_path);

    for (const auto &agent : config)
    {
      TransformStamped transform;
      transform.header.frame_id = leader_frame_; 
      transform.child_frame_id = agent.first.as<string>(); // Child frame (e.g., Agent_0, Agent_1)

      transform.transform.translation.x = agent.second["x"].as<double>();
      transform.transform.translation.y = agent.second["y"].as<double>();
      transform.transform.translation.z = agent.second["z"].as<double>();

      // No rotation
      transform.transform.rotation.x = 0.0;
      transform.transform.rotation.y = 0.0;
      transform.transform.rotation.z = 0.0;
      transform.transform.rotation.w = 1.0;

      transforms_.push_back(transform);
    }

    RCLCPP_INFO(this->get_logger(), "Loaded %zu transforms from YAML file.", transforms_.size());
  }
  catch (const YAML::Exception &e)
  {
    RCLCPP_ERROR(this->get_logger(), "Failed to load YAML file: %s", e.what());
  }
}

void FormationShapeBroadcaster::publish_transforms()
{
  // Update the timestamp for each transform and publish
  for (auto &transform : transforms_)
  {
    transform.header.stamp = this->get_clock()->now();
  }
  static_broadcaster_->sendTransform(transforms_);
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<FormationShapeBroadcaster>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}