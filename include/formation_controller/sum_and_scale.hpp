#ifndef SUM_AND_SCALE_HPP
#define SUM_AND_SCALE_HPP

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"

using namespace std;
using Float32 = std_msgs::msg::Float32;

class SumAndScale : public rclcpp::Node
{
public:
  /*
  Take in normalized control values from two topics
  Sum them up, restrict them to (-1, 1)
  Multiply by scaling constant and publish */
  SumAndScale();

private:
  rclcpp::Subscription<Float32>::SharedPtr sub1_;
  rclcpp::Subscription<Float32>::SharedPtr sub2_;
  rclcpp::Publisher<Float32>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  void timer_cb();

  float value1_ = 0.;
  float value2_ = 0.;
  float scaling_factor_;
  bool is_new_value1_ = false;
  bool is_new_value2_ = false;
};

#endif // SUM_AND_SCALE_HPP