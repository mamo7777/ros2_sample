// ROS2 training program (C++)
// Mamoru Uchiuda(Meitec)
#include <chrono>
#include <string>

#include "rclcpp/rclcpp.hpp"

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
  void vel1_callback(const std::shared_ptr<geometry_msgs::msg::Twist> msg);

  // メンバー変数
  uint32_t counter_ = 0;

  // rclcpp
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel1_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel2_publisher_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr turtle_1_cmd_vel_sub_;
};

// コンストラクタ
CppTestNode::CppTestNode() : Node("cpp_test_node") {
  using namespace std::literals::chrono_literals;
  // タイマーコールバックの初期化
  timer_ = this->create_wall_timer(1000ms, std::bind(&CppTestNode::timer_callback, this));

  // 速度指令パブリッシャーの初期化
  vel1_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/turtle1/cmd_vel", 10);
  vel2_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/turtle2/cmd_vel", 10);

  // 速度指令サブスクライバーの初期化
  turtle_1_cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "/turtle1/cmd_vel", 10, std::bind(&CppTestNode::vel1_callback, this, std::placeholders::_1));
}

// タイマーコールバック
void CppTestNode::timer_callback() {
  if (counter_ >= 100) {
    // Errorメッセージ
    RCLCPP_ERROR(this->get_logger(), "[%d]Hello, World from CPP!", counter_);
  } else if (counter_ >= 50) {
    // Warningメッセージ
    RCLCPP_WARN(this->get_logger(), "[%d]Hello, World from CPP!", counter_);
  } else {
    // Infoメッセージ
    RCLCPP_INFO(this->get_logger(), "[%d]Hello, World from CPP!", counter_);
  }
  ++counter_;

  // 速度指令(turtle1)のパブリッシュ
  auto message = geometry_msgs::msg::Twist();
  message.linear.x = 1.0;
  message.angular.z = 1.0;
  vel1_publisher_->publish(message);
}

// 速度指令トピックのコールバック関数
void CppTestNode::vel1_callback(const std::shared_ptr<geometry_msgs::msg::Twist> msg) {
  // メッセージから指令値生成
  auto vel2_msg = geometry_msgs::msg::Twist();
  vel2_msg.linear.x = msg->linear.x * 2;
  vel2_msg.angular.z = -msg->angular.z;
  // 速度指令(turtle2)のパブリッシュ
  vel2_publisher_->publish(vel2_msg);
}

// main関数
int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);

  auto node = std::make_shared<CppTestNode>();

  rclcpp::spin(node);

  rclcpp::shutdown();

  return 0;
}
