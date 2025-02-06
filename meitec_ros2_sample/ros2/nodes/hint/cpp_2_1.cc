// ROS training program (C++)
// Mamoru Uchiuda(Meitec)
#include <chrono>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "spdlog/spdlog.h"

// https://github.com/ros2/common_interfaces/tree/jazzy
#include <geometry_msgs/msg/twist.hpp>

// クラス
class CppTestNode : public rclcpp::Node {
 public:
  CppTestNode();
  ~CppTestNode() = default;

 private:
  // メンバー関数
  void timer_callback();

  // メンバー変数
  uint64_t counter_ = 0;

  // ROS
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_publisher_;
};

// コンストラクタ
CppTestNode::CppTestNode() : Node("cpp_test_node") {
  using namespace std::literals::chrono_literals;
  // タイマーコールバックの初期化
  timer_ = this->create_wall_timer(1000ms, std::bind(&CppTestNode::timer_callback, this));

  // 速度指令パブリッシャーの初期化
  vel_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/turtle1/cmd_vel", 10);
}

// タイマーコールバック
void CppTestNode::timer_callback() {
  if (counter_ >= 100) {
    // Errorメッセージ
    SPDLOG_ERROR("[{}]Hello, World from CPP!", counter_);
  } else if (counter_ >= 50) {
    // Warningメッセージ
    SPDLOG_WARN("[{}]Hello, World from CPP!", counter_);
  } else {
    // Infoメッセージ
    SPDLOG_INFO("[{}]Hello, World from CPP!", counter_);
  }
  ++counter_;

  // 速度指令(turtle1)のパブリッシュ
  auto message = geometry_msgs::msg::Twist();
  message.linear.x = 1.0;
  message.angular.z = 1.0;
  vel_publisher_->publish(message);
}

// main関数
int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);

  auto node = std::make_shared<CppTestNode>();

  rclcpp::spin(node);

  rclcpp::shutdown();

  return 0;
}
