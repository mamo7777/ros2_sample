#!/usr/bin/env python3
# ROS2 training program (Python)
# Mamoru Uchiuda(Meitec)

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist


# クラス
class PyTestNode(Node):
    # コンストラクタ
    def __init__(self) -> None:
        super().__init__("py_test_node")

        # インスタンス変数
        self._counter = 0

        # タイマーコールバックの初期化
        timer_period = 1.0
        self._timer = self.create_timer(timer_period, self._timer_callback)

        # 速度指令パブリッシャーの初期化
        self._vel1_publisher = self.create_publisher(Twist, "/turtle1/cmd_vel", 10)
        self._vel2_publisher = self.create_publisher(Twist, "/turtle2/cmd_vel", 10)

        # 速度指令サブスクライバーの初期化
        self._turtle_1_cmd_vel_sub = self.create_subscription(
            Twist, "/turtle1/cmd_vel", self._vel1_callback, 10
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

        # 速度指令(turtle1)のパブリッシュ
        message = Twist()
        message.linear.x = 1.0
        message.angular.z = 1.0
        self._vel1_publisher.publish(message)

    # 速度指令トピックのコールバック関数
    def _vel1_callback(self, msg) -> None:
        # メッセージから指令値生成
        vel2_msg = Twist()
        vel2_msg.linear.x = msg.linear.x * 2
        vel2_msg.angular.z = -msg.angular.z
        # 速度指令(turtle2)のパブリッシュ
        self._vel2_publisher.publish(vel2_msg)


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
