import rclpy
from rclpy.node import Node


class ExploreNode(Node):
    def __init__(self):
        super().__init__("explore_node")
        self.get_logger().info("ExploreLite placeholder node started")


def main(args=None):
    rclpy.init(args=args)
    node = ExploreNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
