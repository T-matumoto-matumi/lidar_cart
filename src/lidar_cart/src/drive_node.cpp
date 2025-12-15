#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"

using namespace std::chrono_literals;

class DriveNode : public rclcpp::Node
{
public:
  DriveNode()
  : Node("drive_node")
  {
    publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    timer_ = this->create_wall_timer(
      100ms, std::bind(&DriveNode::timer_callback, this));
    RCLCPP_INFO(this->get_logger(), "Drive Node Started. Robot should move in a circle.");
  }

private:
  void timer_callback()
  {
    auto message = geometry_msgs::msg::Twist();
    message.linear.x = 0.5;
    message.angular.z = 0.5;
    publisher_->publish(message);
  }
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DriveNode>());
  rclcpp::shutdown();
  return 0;
}
