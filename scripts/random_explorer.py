#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from action_msgs.msg import GoalStatus
from nav_msgs.msg import OccupancyGrid
import numpy as np
import random
import math


class RandomExplorer(Node):
    def __init__(self):
        super().__init__('random_explorer')

        # (Opcional) límites por si quieres restringir el área
        self.declare_parameter('x_min', -2.0)
        self.declare_parameter('x_max',  2.0)
        self.declare_parameter('y_min', -2.0)
        self.declare_parameter('y_max',  2.0)

        # Action server de Nav2
        self._client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.goal_active = False

        # --- NUEVO: suscripción al costmap global ---
        self.costmap_sub = self.create_subscription(
            OccupancyGrid,
            '/global_costmap/costmap',          # mismo que usa Nav2
            self._costmap_cb,
            10)

        self.free_cells = []      # [(fila, col), ...] cuya prob. < 10
        self.map_info   = None    # nav_msgs/MapMetaData

        # Temporizador que intenta lanzar un goal cada segundo
        self.timer = self.create_timer(1.0, self._send_goal)

    # ---------- callbacks ----------
    def _costmap_cb(self, msg: OccupancyGrid):
        """Guarda las celdas libres del costmap."""
        data = np.array(msg.data, dtype=np.int8).reshape(msg.info.height,
                                                         msg.info.width)
        free = np.where(data < 10)           # coste muy bajo ≈ libre
        self.free_cells = list(zip(free[0], free[1]))
        self.map_info   = msg.info

    def _sample_free_pose(self):
        """Devuelve (x, y) en el centro de una celda libre aleatoria."""
        r, c = random.choice(self.free_cells)
        x = c * self.map_info.resolution + self.map_info.origin.position.x \
            + self.map_info.resolution / 2.0
        y = r * self.map_info.resolution + self.map_info.origin.position.y \
            + self.map_info.resolution / 2.0
        return x, y

    # ---------- gestión del goal ----------
    def _send_goal(self):
        if not self._client.server_is_ready():
            self.get_logger().info('Esperando a navigate_to_pose…')
            return
        if self.goal_active:
            return
        if not self.free_cells:
            self.get_logger().warn('Aún no he recibido el costmap…')
            return

        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp    = self.get_clock().now().to_msg()

        pose.pose.position.x, pose.pose.position.y = self._sample_free_pose()

        yaw = random.uniform(-math.pi, math.pi)
        pose.pose.orientation.z = math.sin(yaw / 2.0)
        pose.pose.orientation.w = math.cos(yaw / 2.0)

        goal = NavigateToPose.Goal()
        goal.pose = pose

        self.goal_active = True
        self._client.send_goal_async(goal).add_done_callback(self._goal_response)

    def _goal_response(self, future):
        gh = future.result()
        if not gh.accepted:
            self.get_logger().info('Goal rechazado')
            self.goal_active = False
            return
        gh.get_result_async().add_done_callback(self._result_callback)

    def _result_callback(self, future):
        result = future.result()
        if result.status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('Goal alcanzado')
        else:
            self.get_logger().info(f'Goal terminó con status {result.status}')
        self.goal_active = False


def main(args=None):
    rclpy.init(args=args)
    node = RandomExplorer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

