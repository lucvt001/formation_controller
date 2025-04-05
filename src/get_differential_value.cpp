#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>

using namespace std;
using Float32 = std_msgs::msg::Float32;

class DifferentialValueNode : public rclcpp::Node
{
public:
  DifferentialValueNode()
  : Node("differential_value_node")
  {
    string input_topic = this->declare_parameter<string>("input_topic", "");
    string output_topic = this->declare_parameter<string>("output_topic", "");
    dt_ = this->declare_parameter<float>("dt", 0.5);

    input_sub_ = this->create_subscription<Float32>(
      input_topic, 10, std::bind(&DifferentialValueNode::topic_callback, this, std::placeholders::_1));

    pub_ = this->create_publisher<Float32>(output_topic, 10);
  }

private:
  void topic_callback(const Float32::SharedPtr msg)
  {
    float current_value = msg->data;
    if (abs(prev_value_ - (-1.0)) < 1e-6) // first value
    {
      prev_value_ = current_value;
      return;
    }
    float rate_of_change = (current_value - prev_value_) / dt_;
    if (abs(rate_of_change) < 1e-6) return;
    auto output_msg = Float32();
    output_msg.data = rate_of_change;
    pub_->publish(output_msg);
    prev_value_ = current_value;
  }

  float prev_value_ = -1.0;
  float dt_;

  rclcpp::Subscription<Float32>::SharedPtr input_sub_;
  rclcpp::Subscription<Float32>::SharedPtr rpm_float_sub_;
  rclcpp::Publisher<Float32>::SharedPtr pub_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DifferentialValueNode>());
  rclcpp::shutdown();
  return 0;
}