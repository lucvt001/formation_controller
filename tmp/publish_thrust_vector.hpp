#include "behaviortree_ros2/bt_topic_pub_node_async.hpp"
#include "sam_msgs/msg/thruster_angles.hpp"

using namespace BT;
using ThrusterAngles = sam_msgs::msg::ThrusterAngles;

class PublishThrustVector : public AsyncRosTopicPubNode<ThrusterAngles>
{

public:
  PublishThrustVector(const std::string& name, const NodeConfig& conf, const RosNodeParams& params)
    : AsyncRosTopicPubNode<ThrusterAngles>(name, conf, params)
  {}

  static PortsList providedPorts()
  {
    return providedBasicPorts({ 
      InputPort<float>("horizontal_radians", 0.0, "Between -0.2 to 0.2 radians"),
      InputPort<float>("vertical_radians", 0.0, "Between -0.2 to 0.2 radians")
    });
  }

  bool setMessage(ThrusterAngles& msg) override
  {
    getInput("horizontal_radians", msg.thruster_horizontal_radians);
    getInput("vertical_radians", msg.thruster_vertical_radians);
    return true;
  }
};