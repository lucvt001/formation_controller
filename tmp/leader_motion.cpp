#include <rclcpp/rclcpp.hpp>
#include <smarc_msgs/msg/thruster_rpm.hpp>
#include <chrono>

using namespace std;
using namespace std::chrono_literals;
using ThrusterRPM = smarc_msgs::msg::ThrusterRPM;

class LeaderMotion : public rclcpp::Node
{
public:
    LeaderMotion() : Node("leader_motion"), stop_(false)
    {
        this->declare_parameter<string>("leader1_thruster1_topic", "");
        this->declare_parameter<string>("leader1_thruster2_topic", "");
        this->declare_parameter<string>("leader2_thruster1_topic", "");
        this->declare_parameter<string>("leader2_thruster2_topic", "");
        this->declare_parameter<int>("rpm", 500);
        this->declare_parameter<float>("duration", 20.0);

        string leader1_thruster1_topic = this->get_parameter("leader1_thruster1_topic").as_string();
        string leader1_thruster2_topic = this->get_parameter("leader1_thruster2_topic").as_string();
        string leader2_thruster1_topic = this->get_parameter("leader2_thruster1_topic").as_string();
        string leader2_thruster2_topic = this->get_parameter("leader2_thruster2_topic").as_string();
        rpm_ = this->get_parameter("rpm").as_int();
        float duration = this->get_parameter("duration").as_double();

        publisher11_ = this->create_publisher<ThrusterRPM>(leader1_thruster1_topic, 10);
        publisher12_ = this->create_publisher<ThrusterRPM>(leader1_thruster2_topic, 10);
        publisher21_ = this->create_publisher<ThrusterRPM>(leader2_thruster1_topic, 10);
        publisher22_ = this->create_publisher<ThrusterRPM>(leader2_thruster2_topic, 10);

        timer_ = this->create_wall_timer(
            50ms, std::bind(&LeaderMotion::publishCommand, this));

        stop_timer_ = this->create_wall_timer(
            std::chrono::duration<double>(duration), std::bind(&LeaderMotion::stopFleet, this));
    }

private:
    void publishCommand()
    {
        if (stop_)
        {
            return;
        }

        auto message = ThrusterRPM();
        message.rpm = rpm_;

        publisher11_->publish(message);
        publisher12_->publish(message);
        publisher22_->publish(message);
        publisher21_->publish(message);
    }

    void stopFleet()
    {
        stop_ = true;
        auto message = ThrusterRPM();
        message.rpm = 0;

        publisher11_->publish(message);
        publisher12_->publish(message);
        publisher22_->publish(message);
        publisher21_->publish(message);

        RCLCPP_INFO(this->get_logger(), "Fleet stopped.");
    }

    rclcpp::Publisher<ThrusterRPM>::SharedPtr publisher11_;
    rclcpp::Publisher<ThrusterRPM>::SharedPtr publisher12_;
    rclcpp::Publisher<ThrusterRPM>::SharedPtr publisher21_;
    rclcpp::Publisher<ThrusterRPM>::SharedPtr publisher22_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::TimerBase::SharedPtr stop_timer_;
    int rpm_;
    bool stop_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LeaderMotion>());
    rclcpp::shutdown();
    return 0;
}