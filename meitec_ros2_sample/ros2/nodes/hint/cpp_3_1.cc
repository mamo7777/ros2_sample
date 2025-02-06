#include <chrono>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "spdlog/spdlog.h"

// https://github.com/ros2/common_interfaces/tree/jazzy
#include <geometry_msgs/msg/twist.hpp>

class CppTestNode : public rclcpp::Node {
 public:
  CppTestNode();
  ~CppTestNode() = default;

 private:
  void timer_callback();
  void vel1_callback(const std::shared_ptr<geometry_msgs::msg::Twist> msg);
  uint64_t counter_ = 0;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel1_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel2_publisher_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr turtle_1_cmd_vel_sub_;
};

CppTestNode::CppTestNode() : Node("cpp_test_node") {
  using namespace std::literals::chrono_literals;
  timer_ = this->create_wall_timer(1000ms, std::bind(&CppTestNode::timer_callback, this));

  vel1_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/turtle1/cmd_vel", 10);
  vel2_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/turtle2/cmd_vel", 10);

  turtle_1_cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "/turtle1/cmd_vel", 10, std::bind(&CppTestNode::vel1_callback, this, std::placeholders::_1));
}

void CppTestNode::timer_callback() {
  if (counter_ >= 100) {
    SPDLOG_ERROR("[{}]Hello, World from CPP!", counter_);
    rclcpp::shutdown();
    return;
  } else if (counter_ >= 50) {
    SPDLOG_WARN("[{}]Hello, World from CPP!", counter_);
  } else {
    SPDLOG_INFO("[{}]Hello, World from CPP!", counter_);
  }
  ++counter_;

  auto message = geometry_msgs::msg::Twist();
  message.linear.x = 1.0;
  message.angular.z = 1.0;
  vel1_publisher_->publish(message);
}

void CppTestNode::vel1_callback(const std::shared_ptr<geometry_msgs::msg::Twist> msg) {
  auto vel2_msg = geometry_msgs::msg::Twist();
  vel2_msg.linear.x = msg->linear.x * 2;
  vel2_msg.angular.z = -msg->angular.z;
  vel2_publisher_->publish(vel2_msg);
}

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);

  auto node = std::make_shared<CppTestNode>();

  rclcpp::spin(node);

  return 0;
}
