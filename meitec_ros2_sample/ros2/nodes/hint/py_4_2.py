#!/usr/bin/env python3
# ROS2 training program (Python)
# Mamoru Uchiuda(Meitec)

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from std_srvs.srv import Trigger, SetBool


# クラス
class PyTestNode(Node):
    # コンストラクタ
    def __init__(self) -> None:
        super().__init__("py_test_node")

        # インスタンス変数
        self._counter = 0
        self._turn_direction = 1
        self._operation = True

        # タイマーコールバックの初期化
        timer_period = 1.0
        self._timer = self.create_timer(timer_period, self._timer_callback)

        # 速度指令パブリッシャーの初期化
        self._vel1_publisher = self.create_publisher(Twist, "/turtle1/cmd_vel", 10)
        self._vel2_publisher = self.create_publisher(Twist, "/turtle2/cmd_vel", 10)

        # 速度指令サブスクライバーの初期化
        self.turtle_1_cmd_vel_sub = self.create_subscription(
            Twist, "/turtle1/cmd_vel", self._vel1_callback, 10
        )

        # 向き変更サービスの初期化
        self._change_direction_service = self.create_service(
            Trigger, "/turtle1/change_direction", self._chg_dir_srv_callback
        )

        # 停止サービスの初期化
        self._turtle_operation_service = self.create_service(
            SetBool, "/turtle_pub_operation", self._operation_srv_callback
        )

    # タイマーコールバック
    def _timer_callback(self) -> None:
        # Errorメッセージ
        if self._counter >= 100:
            self.get_logger().error(f"[{self._counter}] Hello, World from PY!")
        # Warningメッセージ
        elif self._counter >= 50:
            self.get_logger().warn(f"[{self._counter}] Hello, World from PY!")
        # Infoメッセージ
        else:
            self.get_logger().info(f"[{self._counter}] Hello, World from PY!")

        self._counter = self._counter + 1

        if self._operation:
            # 速度指令(turtle1)のパブリッシュ
            message = Twist()
            message.linear.x = 1.0
            message.angular.z = 1.0 * self._turn_direction
            self._vel1_publisher.publish(message)

    # 速度指令トピックのコールバック関数
    def _vel1_callback(self, msg) -> None:
        # メッセージから指令値生成
        vel2_msg = Twist()
        vel2_msg.linear.x = msg.linear.x * 2
        vel2_msg.angular.z = -msg.angular.z
        # 速度指令(turtle2)のパブリッシュ
        self._vel2_publisher.publish(vel2_msg)

    # 向き変更サービスのコールバック関数
    def _chg_dir_srv_callback(self, req, res) -> Trigger.Response:
        # 向きの反転
        self._turn_direction = self._turn_direction * -1

        # Response作成
        res.success = True
        res.message = "[PY]Success!"

        return res

    # オペレーションサービスコールバック関数
    def _operation_srv_callback(self, req, res) -> SetBool.Response:
        res_msg = ""

        # リクエストに応じて処理を変える
        if req.data:
            res_msg = "Start Operation Success"
            self._operation = True
        else:
            res_msg = "Stop Operation Success"
            self._operation = False

        # Response作成
        res.success = True
        res.message = res_msg

        return res


# main関数
def main(args=None) -> None:
    rclpy.init(args=args)

    node = PyTestNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().error("User interrupt")
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == "__main__":
    main()
