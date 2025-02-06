#include <chrono>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "spdlog/spdlog.h"

// https://github.com/ros2/common_interfaces/tree/jazzy
#include <geometry_msgs/msg/twist.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <turtlesim/srv/spawn.hpp>

class CppTestNode : public rclcpp::Node {
 public:
  CppTestNode();
  ~CppTestNode() = default;

 private:
  void timer_callback();
  void vel1_callback(const std::shared_ptr<geometry_msgs::msg::Twist> msg);
  void chg_dir_srv_callback(const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
                            const std::shared_ptr<std_srvs::srv::Trigger::Response> res);

  bool check_turtle2();
  void spawn_turtle2();

  uint64_t counter_ = 0;
  int8_t turn_direction_ = 1;

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel1_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel2_publisher_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr turtle_1_cmd_vel_sub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr change_direction_service_;
  rclcpp::Client<turtlesim::srv::Spawn>::SharedPtr spawn_cli_;
};

CppTestNode::CppTestNode() : Node("cpp_test_node") {
  using namespace std::literals::chrono_literals;
  timer_ = this->create_wall_timer(1000ms, std::bind(&CppTestNode::timer_callback, this));

  vel1_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/turtle1/cmd_vel", 10);
  vel2_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/turtle2/cmd_vel", 10);

  turtle_1_cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "/turtle1/cmd_vel", 10, std::bind(&CppTestNode::vel1_callback, this, std::placeholders::_1));

  change_direction_service_ = create_service<std_srvs::srv::Trigger>(
      "/turtle1/change_direction", std::bind(&CppTestNode::chg_dir_srv_callback, this,
                                             std::placeholders::_1, std::placeholders::_2));

  spawn_cli_ = this->create_client<turtlesim::srv::Spawn>("/spawn");
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
  message.angular.z = 1.0 * turn_direction_;
  vel1_publisher_->publish(message);

  if (!check_turtle2()) {
    spawn_turtle2();
  }
}

void CppTestNode::vel1_callback(const std::shared_ptr<geometry_msgs::msg::Twist> msg) {
  auto vel2_msg = geometry_msgs::msg::Twist();
  vel2_msg.linear.x = msg->linear.x * 2;
  vel2_msg.angular.z = -msg->angular.z;
  vel2_publisher_->publish(vel2_msg);
}

void CppTestNode::chg_dir_srv_callback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> /*req*/,
    const std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
  turn_direction_ *= -1;
  res->success = true;
  res->message = "Success";
}

bool CppTestNode::check_turtle2() {
  auto topic_names = this->get_topic_names_and_types();
  for (const auto& topic : topic_names) {
    if (topic.first == "/turtle2/pose") {
      return true;
    }
  }
  return false;
}

void CppTestNode::spawn_turtle2() {
  auto request = std::make_shared<turtlesim::srv::Spawn::Request>();
  request->x = 2.5;
  request->y = 4.0;
  request->theta = 0.0;
  request->name = "turtle2";

  while (!spawn_cli_->wait_for_service(std::chrono::seconds(1))) {
    SPDLOG_WARN("Waiting for /spawn service...");
  }

  auto result = spawn_cli_->async_send_request(request);
  if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result) ==
      rclcpp::FutureReturnCode::SUCCESS) {
    SPDLOG_INFO("Spawned turtle2 successfully.");
  } else {
    SPDLOG_ERROR("Failed to spawn turtle2.");
  }
}

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);

  auto node = std::make_shared<CppTestNode>();

  rclcpp::spin(node);

  return 0;
}
