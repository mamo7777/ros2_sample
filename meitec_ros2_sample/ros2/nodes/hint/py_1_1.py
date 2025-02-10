#!/usr/bin/env python3

import rclpy
from rclpy.node import Node


class PyTestNode(Node):
    def __init__(self) -> None:
        super().__init__("py_test_node")

        self.timer_period = 1.0
        self.counter = 0

        self.timer = self.create_timer(self.timer_period, self._timer_callback)

    def _timer_callback(self) -> None:
        if self.counter >= 10:
            self.get_logger().error(f"[{self.counter}] Hello, World from PY!")
            self.destroy_node()
            rclpy.shutdown()
        elif self.counter >= 5:
            self.get_logger().warn(f"[{self.counter}] Hello, World from PY!")
        else:
            self.get_logger().info(f"[{self.counter}] Hello, World from PY!")

        self.counter = self.counter + 1


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
