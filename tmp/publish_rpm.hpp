#include "behaviortree_ros2/bt_topic_pub_node_async.hpp"
#include "smarc_msgs/msg/thruster_rpm.hpp"

using namespace BT;
using ThrusterRPM = smarc_msgs::msg::ThrusterRPM;

class PublishRPM : public AsyncRosTopicPubNode<ThrusterRPM>
{

public:
  PublishRPM(const std::string& name, const NodeConfig& conf, const RosNodeParams& params)
    : AsyncRosTopicPubNode<ThrusterRPM>(name, conf, params) {}

  static PortsList providedPorts()
  {
    return providedBasicPorts({ 
      InputPort<int>("rpm", 0, "Thruster rpm to be published (between 0 and 1000)")
    });
  }

  bool setMessage(ThrusterRPM& msg) override
  {
    getInput("rpm", msg.rpm);
    return true;
  }
};