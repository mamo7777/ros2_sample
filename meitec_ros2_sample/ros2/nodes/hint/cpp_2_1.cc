#include <chrono>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "spdlog/spdlog.h"

#include <geometry_msgs/msg/twist.hpp>

class CppTestNode : public rclcpp::Node {
 public:
  CppTestNode();
  ~CppTestNode() = default;

 private:
  void timer_callback();
  uint64_t counter_ = 0;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_publisher_;
};

CppTestNode::CppTestNode() : Node("cpp_test_node") {
  using namespace std::literals::chrono_literals;
  timer_ = this->create_wall_timer(1000ms, std::bind(&CppTestNode::timer_callback, this));

  vel_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/turtle1/cmd_vel", 10);
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
  vel_publisher_->publish(message);
}

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);

  auto node = std::make_shared<CppTestNode>();

  rclcpp::spin(node);

  return 0;
}
