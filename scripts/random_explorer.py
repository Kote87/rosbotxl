#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from nav2_msgs.srv import ComputePathToPose
from rclpy.action import ActionClient
import random
import math

class RandomExplorer(Node):
    def __init__(self):
        super().__init__('random_explorer')
        self._client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self._planner = self.create_client(ComputePathToPose, 'compute_path_to_pose')
        self.goal_active = False
        self.timer = self.create_timer(1.0, self._send_goal)

    def _send_goal(self):
        if not self._client.server_is_ready():
            self.get_logger().info('Waiting for navigate_to_pose action server...')
            return
        if not self._planner.wait_for_service(timeout_sec=0.1):
            self.get_logger().info('Waiting for compute_path_to_pose service...')
            return
        if self.goal_active:
            return

        for _ in range(5):
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.pose.position.x = random.uniform(-2.0, 2.0)
            pose.pose.position.y = random.uniform(-2.0, 2.0)
            yaw = random.uniform(-math.pi, math.pi)
            pose.pose.orientation.z = math.sin(yaw / 2.0)
            pose.pose.orientation.w = math.cos(yaw / 2.0)

            req = ComputePathToPose.Request()
            req.goal = pose
            req.use_start = False
            future = self._planner.call_async(req)
            rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
            result = future.result()
            if result and result.path.poses:
                goal = NavigateToPose.Goal()
                goal.pose = pose
                self.goal_active = True
                self._client.send_goal_async(goal).add_done_callback(self._goal_response)
                return

        self.get_logger().info('Failed to find a valid goal')

    def _goal_response(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected')
            self.goal_active = False
            return
        goal_handle.get_result_async().add_done_callback(self._result_callback)

    def _result_callback(self, future):
        self.get_logger().info('Goal completed')
        self.goal_active = False


def main(args=None):
    rclpy.init(args=args)
    node = RandomExplorer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
