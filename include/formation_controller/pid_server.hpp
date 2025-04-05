#ifndef PID_CONTROL_HPP
#define PID_CONTROL_HPP

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <tuper_interfaces/action/run_pid.hpp>
#include <std_msgs/msg/float32.hpp>
#include "pid.h"

using namespace std;
using namespace rclcpp_action;
using Float32 = std_msgs::msg::Float32;
using RunPid = tuper_interfaces::action::RunPid;

class PidServer : public rclcpp::Node 
{

public:
  PidServer();

private:
  rclcpp::Subscription<Float32>::SharedPtr input_sub_;
  rclcpp::Publisher<Float32>::SharedPtr output_pub_;
  Server<RunPid>::SharedPtr action_server_;
  void execute(const std::shared_ptr<ServerGoalHandle<RunPid>> goal_handle);

  // A very important flag to ensure that we are always processing new messages in each loop
  bool is_new_msg_received_ = false;
  float current_;

  void initializePidController();
  PID pid_ = PID(0,0,0,0,0,0);
  int is_axis_inverted_;
};

#endif // PID_CONTROL_HPP
