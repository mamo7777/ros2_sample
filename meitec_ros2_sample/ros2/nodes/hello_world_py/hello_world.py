#!/usr/bin/env python3
# ROS2 training program (Python)
# Mamoru Uchiuda(Meitec)

import rclpy
from rclpy.node import Node


# クラス
class PyTestNode(Node):
    # コンストラクタ
    def __init__(self) -> None:
        super().__init__("py_test_node")

        self._timer_period = 1.0
        self._counter = 0

        # タイマーコールバックの初期化
        self._timer = self.create_timer(self._timer_period, self._timer_callback)

    # タイマーコールバック
    def _timer_callback(self) -> None:
        # Errorメッセージ
        if self._counter >= 10:
            self.get_logger().error(f"[{self._counter}] Hello, World from PY!")
            self.destroy_node()
            rclpy.shutdown()
            return
        # Warningメッセージ
        elif self._counter >= 5:
            self.get_logger().warn(f"[{self._counter}] Hello, World from PY!")
        # Infoメッセージ
        else:
            self.get_logger().info(f"[{self._counter}] Hello, World from PY!")

        self._counter = self._counter + 1


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
