#include "formation_controller/pid_server.hpp"

PidServer::PidServer() : Node("pid_server")
{
  string action_server_name = this->get_name();
  action_server_ = create_server<RunPid>(
    this, action_server_name,
    [this](const GoalUUID & uuid, std::shared_ptr<const RunPid::Goal> goal) {
      (void)uuid; (void)goal;
      return GoalResponse::ACCEPT_AND_EXECUTE;
    },
    [this](const std::shared_ptr<ServerGoalHandle<RunPid>> goal_handle) {
      (void)goal_handle;
      return CancelResponse::ACCEPT;
    },
    [this](const std::shared_ptr<ServerGoalHandle<RunPid>> goal_handle) {
      std::thread([this, goal_handle]() {
        execute(goal_handle);
      }).detach();
    }
  );

  string input_topic = this->declare_parameter("input_topic", "");
  string output_topic = this->declare_parameter("output_topic", "");
  input_sub_ = this->create_subscription<Float32>(input_topic, 10, 
    [this](const Float32::SharedPtr msg) { current_ = msg->data; is_new_msg_received_ = true; });
  output_pub_ = this->create_publisher<Float32>(output_topic, 10);
    
  is_axis_inverted_ = this->declare_parameter("is_axis_inverted", 1);
  initializePidController();
}

void PidServer::execute(const std::shared_ptr<ServerGoalHandle<RunPid>> goal_handle) 
{
  auto goal = goal_handle->get_goal();
  auto feedback = std::make_shared<RunPid::Feedback>();
  auto result = std::make_shared<RunPid::Result>();
  float setpoint = goal->setpoint;

  RCLCPP_DEBUG(this->get_logger(), "PidServer is running");

  pid_.reset();
  rclcpp::Rate rate(10);

  while (rclcpp::ok()) 
  {
    rate.sleep();

    // Check if the goal is being cancelled
    if (goal_handle->is_canceling())
    {
      RCLCPP_DEBUG(this->get_logger(), "PidServer cancelled.");      
      goal_handle->canceled(result);
      return;
    }

    if (!is_new_msg_received_) continue;

    float control_signal = is_axis_inverted_ * pid_.calculate(setpoint, current_);
    output_pub_->publish(Float32().set__data(control_signal));
    
    is_new_msg_received_ = false;
  }
}

void PidServer::initializePidController() 
{
  float kp = this->declare_parameter("kp", 0.0);
  float ki = this->declare_parameter("ki", 0.0);
  float kd = this->declare_parameter("kd", 0.0);
  float dt = this->declare_parameter("dt", 0.1);
  float max = this->declare_parameter("max", 1.0);
  float min = this->declare_parameter("min", -1.0);
  float unwinding_factor = this->declare_parameter("unwinding_factor", 1.0);
  bool is_verbose = this->declare_parameter("is_verbose", false);
  
  pid_ = PID(kp, ki, kd, dt, max, min);
  pid_.set_unwinding_factor(unwinding_factor);
  string node_name = this->get_name();
  pid_.enable_verbose_mode(is_verbose, node_name);
  RCLCPP_INFO(this->get_logger(), "PID initialized with kp: %f, ki: %f, kd: %f, dt: %f, max: %f, min: %f", kp, ki, kd, dt, max, min);
}

int main(int argc, char** argv) 
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PidServer>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
