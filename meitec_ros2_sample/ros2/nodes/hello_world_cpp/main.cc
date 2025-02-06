// ROS training program (C++)
// Mamoru Uchiuda(Meitec)
#include <chrono>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "spdlog/spdlog.h"

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
};

// コンストラクタ
CppTestNode::CppTestNode() : Node("cpp_test_node") {
  using namespace std::literals::chrono_literals;
  // タイマーコールバックの初期化
  timer_ = this->create_wall_timer(1000ms, std::bind(&CppTestNode::timer_callback, this));
}

// タイマーコールバック
void CppTestNode::timer_callback() {
  if (counter_ >= 10) {
    // Errorメッセージ
    SPDLOG_ERROR("[{}]Hello, World from CPP!", counter_);
    rclcpp::shutdown();
    return;
  } else if (counter_ >= 5) {
    // Warningメッセージ
    SPDLOG_WARN("[{}]Hello, World from CPP!", counter_);
  } else {
    // Infoメッセージ
    SPDLOG_INFO("[{}]Hello, World from CPP!", counter_);
  }
  ++counter_;
}

// main関数
int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);

  auto node = std::make_shared<CppTestNode>();

  rclcpp::spin(node);

  rclcpp::shutdown();

  return 0;
}
