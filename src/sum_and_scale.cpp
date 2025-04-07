#include "formation_controller/sum_and_scale.hpp"

SumAndScale::SumAndScale() : Node("horizontal_thrust_vector_relay")
{
  string input_topic1 = this->declare_parameter<string>("input_topic1", "");
  string input_topic2 = this->declare_parameter<string>("input_topic2", "");
  string output_topic = this->declare_parameter<string>("output_topic", "");
  scaling_factor_ = this->declare_parameter<float>("scaling_factor", 0.1);

  pub_ = this->create_publisher<Float32>(output_topic, 10);

  sub1_ = this->create_subscription<Float32>(
    input_topic1, 10, [this](const Float32::SharedPtr msg) { value1_ = msg->data; is_new_value1_ = true; });

  sub2_ = this->create_subscription<Float32>(
    input_topic2, 10, [this](const Float32::SharedPtr msg) { value2_ = msg->data; is_new_value2_ = true; });

  timer_ = this->create_wall_timer(100ms, std::bind(&SumAndScale::timer_cb, this));   // 10 Hz
}

void SumAndScale::timer_cb()
{
  if (is_new_value1_ && is_new_value2_)
  {
    is_new_value1_ = false;
    is_new_value2_ = false;
    auto message = Float32();
    message.data = min(max(value1_ + value2_, -1.f), 1.f) * scaling_factor_;
    pub_->publish(message);
    // RCLCPP_INFO(this->get_logger(), "Received values: %f, %f. Published value: %f", value1_, value2_, message.data);
  }
}

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SumAndScale>());
  rclcpp::shutdown();
  return 0;
}