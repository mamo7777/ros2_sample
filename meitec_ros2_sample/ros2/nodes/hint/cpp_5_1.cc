// ROS2 training program (C++)
// Mamoru Uchiuda(Meitec)
#include <chrono>
#include <string>

#include "rclcpp/rclcpp.hpp"

// https://github.com/ros2/common_interfaces/tree/jazzy
#include <geometry_msgs/msg/twist.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <turtlesim/srv/spawn.hpp>

// クラス
class CppTestNode : public rclcpp::Node {
 public:
  CppTestNode();
  ~CppTestNode() = default;

 private:
  // メンバー関数
  void timer_callback();
  void vel1_callback(const std::shared_ptr<geometry_msgs::msg::Twist> msg);
  void chg_dir_srv_callback(const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
                            const std::shared_ptr<std_srvs::srv::Trigger::Response> res);
  void operation_srv_callback(const std::shared_ptr<std_srvs::srv::SetBool::Request> req,
                              const std::shared_ptr<std_srvs::srv::SetBool::Response> res);
  bool check_turtle();
  void spawn_turtle();

  // メンバー変数
  uint32_t counter_ = 0;
  int8_t turn_direction_ = 1;
  bool operation_ = true;

  // rclcpp
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel1_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel2_publisher_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr turtle_1_cmd_vel_sub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr change_direction_service_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr turtle_operation_service_;
  rclcpp::Client<turtlesim::srv::Spawn>::SharedPtr spawn_cli_;
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

  // 向き変更サービスの初期化
  change_direction_service_ = create_service<std_srvs::srv::Trigger>(
      "/turtle1/change_direction", std::bind(&CppTestNode::chg_dir_srv_callback, this,
                                             std::placeholders::_1, std::placeholders::_2));
  // 停止サービスの初期化
  turtle_operation_service_ = create_service<std_srvs::srv::SetBool>(
      "/turtle_pub_operation", std::bind(&CppTestNode::operation_srv_callback, this,
                                         std::placeholders::_1, std::placeholders::_2));

  // 亀のスポーンクライアントの初期化
  spawn_cli_ = this->create_client<turtlesim::srv::Spawn>("/spawn");
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

  if (operation_) {
    // 速度指令(turtle1)のパブリッシュ
    auto message = geometry_msgs::msg::Twist();
    message.linear.x = 1.0;
    message.angular.z = 1.0 * turn_direction_;
    vel1_publisher_->publish(message);
  }

  // turtle2が存在しているか確認
  if (!check_turtle()) {
    // turtle2がなければスポーンサービスをCallする
    spawn_turtle();
  }
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

// 向き変更サービスのコールバック関数
void CppTestNode::chg_dir_srv_callback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> /*req*/,
    const std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
  // 向きの反転
  turn_direction_ *= -1;

  // Response作成
  res->success = true;
  res->message = "Success";
}

// オペレーションサービスコールバック関数
void CppTestNode::operation_srv_callback(
    const std::shared_ptr<std_srvs::srv::SetBool::Request> req,
    const std::shared_ptr<std_srvs::srv::SetBool::Response> res) {
  std::string res_msg;

  // リクエストに応じて処理を変える
  if (req->data) {
    res_msg = "Start Operation Success";
    operation_ = true;
  } else {
    res_msg = "Stop Operation Success";
    operation_ = false;
  }

  // Response作成
  res->success = true;
  res->message = res_msg;
}

// turtle2の存在確認関数(trueならturtle2がある)
bool CppTestNode::check_turtle() {
  // トピックの名前リストの作成
  auto topic_names = this->get_topic_names_and_types();
  for (const auto& topic : topic_names) {
    // turtle2のポーズトピックがあるか確認
    if (topic.first == "/turtle2/pose") {
      // poseがあるのでturtle2出現中
      return true;
    }
  }

  // turtle2のposeが見当たらない
  return false;
}

// turtleの追加スポーンクライアント
void CppTestNode::spawn_turtle() {
  auto request = std::make_shared<turtlesim::srv::Spawn::Request>();
  // 追加スポーン時の生成位置
  request->x = 2.5;
  request->y = 4.0;
  request->name = "turtle2";

  // spawnサービスが存在しているか確認
  // turtlesimがないとspawnは存在しない
  while (rclcpp::ok() && !spawn_cli_->wait_for_service(std::chrono::seconds(1))) {
    RCLCPP_WARN(this->get_logger(), "Waiting for /spawn service...");
  }

  // シャットダウンリクエストが出たらなにもせず処理を終了する
  if (!rclcpp::ok()) {
    RCLCPP_WARN(this->get_logger(), "Node is shutting down, aborting spawn request");
    return;
  }

  // サービスレスポンスのコールバック関数
  auto callback_function = [this](rclcpp::Client<turtlesim::srv::Spawn>::SharedFuture response) {
    if (!response.get()->name.empty()) {
      RCLCPP_INFO(this->get_logger(), "Spawned turtle successfully with name: %s",
                  response.get()->name.c_str());
    } else {
      RCLCPP_ERROR(this->get_logger(), "Failed to spawn turtle");
    }
  };
  // サービスのコール
  spawn_cli_->async_send_request(request, callback_function);
}

// main関数
int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);

  auto node = std::make_shared<CppTestNode>();

  rclcpp::spin(node);

  rclcpp::shutdown();

  return 0;
}
