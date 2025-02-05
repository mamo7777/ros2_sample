#include <chrono>
#include <string>

#include "spdlog/spdlog.h"
#include "rclcpp/rclcpp.hpp"

class CppTestNode : public rclcpp::Node {
 public:
  CppTestNode();
  ~CppTestNode() = default;

 private:
  void timer_callback();
  uint64_t counter_ = 0;
  rclcpp::TimerBase::SharedPtr timer_;
};

CppTestNode::CppTestNode() : Node("cpp_test_node") {
  using namespace std::literals::chrono_literals;
  timer_ = this->create_wall_timer(1000ms, std::bind(&CppTestNode::timer_callback, this));
}

void CppTestNode::timer_callback() {
  if (counter_ >= 10) {
    SPDLOG_ERROR("[{}]Hello, World from CPP!", counter_);
    rclcpp::shutdown();
    return;
  } else if(counter_ >= 5) {
    SPDLOG_WARN("[{}]Hello, World from CPP!", counter_);
  } else {
    SPDLOG_INFO("[{}]Hello, World from CPP!", counter_);
  }
  ++counter_;
}

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);

  auto node = std::make_shared<CppTestNode>();

  rclcpp::spin(node);

  return 0;
}
