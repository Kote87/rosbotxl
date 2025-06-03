#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rclpy
from rclpy.node        import Node
from rclpy.action      import ActionClient
from geometry_msgs.msg import PoseStamped, Point, Twist
from nav2_msgs.action  import NavigateToPose, BackUp, Spin
from nav_msgs.msg      import OccupancyGrid, Odometry
from action_msgs.msg   import GoalStatus
import numpy as np
import cv2, random, math


class RandomExplorer(Node):
    # ---------- inicialización ----------
    def __init__(self):
        super().__init__('random_explorer')

        # Action servers Nav2
        self.nav_client    = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.backup_client = ActionClient(self, BackUp,        'back_up')
        self.spin_client   = ActionClient(self, Spin,          'spin')

        # Subscripciones
        self.create_subscription(OccupancyGrid, '/global_costmap/costmap',
                                 self._costmap_cb, 10)
        self.create_subscription(Odometry,        'odom',
                                 self._odom_cb,   20)

        # Buffers costmap
        self.free_cells: list[tuple[int, int]] = []
        self.map_info   = None
        self.dist: np.ndarray | None = None   # distance transform

        # Estado de acción
        self.goal_active  = False
        self.nav_goal_hdl = None
        self.backing_up   = False
        self.spinning     = False
        self.need_spin    = False

        # Parámetros atasco
        self.stuck_dist    = 0.05   # 5 cm
        self.stuck_timeout = 6.0    # 6 s
        self.spin_cooldown = 20.0   # mínimo 20 s entre Spins
        self.last_pose     = None
        self.last_move_t   = self.get_clock().now()
        self.last_spin_t   = self.get_clock().now()

        # Temporizador principal
        self.create_timer(1.0, self._tick)

    # ---------- callbacks ----------
    def _costmap_cb(self, msg: OccupancyGrid):
        """Guarda celdas libres y pre-calcula distance-transform."""
        grid = np.frombuffer(msg.data, dtype=np.int8).reshape(
                   msg.info.height, msg.info.width)
        free = np.where(grid < 10)
        self.free_cells = list(zip(free[0], free[1]))
        self.map_info   = msg.info

        occ = np.where(grid >= 50, 0, 255).astype(np.uint8)
        self.dist = cv2.distanceTransform(occ, cv2.DIST_L2, 3)

    def _odom_cb(self, msg: Odometry):
        p = msg.pose.pose.position
        if self.last_pose:
            if math.hypot(p.x - self.last_pose.x, p.y - self.last_pose.y) > self.stuck_dist:
                self.last_move_t = self.get_clock().now()
        self.last_pose = p

    # ---------- bucle principal ----------
    def _tick(self):
        if self.backing_up or self.spinning:
            return

        now = self.get_clock().now()
        stalled = (self.goal_active and
                   (now - self.last_move_t).nanoseconds * 1e-9 > self.stuck_timeout and
                   (now - self.last_spin_t).nanoseconds * 1e-9 > self.spin_cooldown)

        if stalled:
            self.get_logger().warn('Atasco detectado → BackUp')
            self._do_backup()
            self.last_spin_t = now      # marca el inicio de la secuencia
            return

        if not self.goal_active:
            self._send_new_goal()

    # ---------- navegación ----------
    def _send_new_goal(self):
        if not (self.nav_client.server_is_ready() and self.free_cells and self.dist is not None):
            return

        r, c = max(self.free_cells, key=lambda rc: self.dist[rc])

        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp    = self.get_clock().now().to_msg()
        pose.pose.position.x = c * self.map_info.resolution + \
                               self.map_info.origin.position.x + self.map_info.resolution / 2
        pose.pose.position.y = r * self.map_info.resolution + \
                               self.map_info.origin.position.y + self.map_info.resolution / 2
        yaw = random.uniform(-math.pi, math.pi)
        pose.pose.orientation.z = math.sin(yaw / 2)
        pose.pose.orientation.w = math.cos(yaw / 2)

        goal = NavigateToPose.Goal(); goal.pose = pose
        self.goal_active = True
        self.nav_client.send_goal_async(goal)\
                       .add_done_callback(self._nav_goal_response)

    def _nav_goal_response(self, fut):
        self.nav_goal_hdl = fut.result()
        if not self.nav_goal_hdl or not self.nav_goal_hdl.accepted:
            self.goal_active = False
            return
        self.nav_goal_hdl.get_result_async().add_done_callback(self._nav_result)

    def _nav_result(self, fut):
        if fut.result().status != GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info(f'Goal abortado (status {fut.result().status})')
        self.goal_active = False

    # ---------- BackUp ----------
    def _do_backup(self):
        if not self.backup_client.server_is_ready():
            self.get_logger().warn('Servidor BackUp no listo')
            self._do_spin(angle=math.pi)   # fallback directo a spin
            return
        if self.nav_goal_hdl:
            self.nav_goal_hdl.cancel_goal_async()
        goal = BackUp.Goal()
        goal.target = Point(x=-0.30, y=0.0, z=0.0)
        goal.speed  = 0.10
        goal.time_allowance.sec = 5
        self.need_spin = True
        self.backing_up = True
        self.backup_client.send_goal_async(goal)\
                          .add_done_callback(self._backup_done)

    def _backup_done(self, _):
        self.backing_up = False
        if self.need_spin:
            self.need_spin = False
            self._do_spin(angle=math.pi)   # media vuelta
        else:
            self.goal_active = False

    # ---------- Spin ----------
    def _do_spin(self, angle: float = math.pi):
        if not self.spin_client.server_is_ready():
            self.get_logger().warn('Servidor Spin no listo')
            return
        goal = Spin.Goal()
        goal.target_yaw = angle
        goal.time_allowance.sec = 6
        self.spinning = True
        self.spin_client.send_goal_async(goal)\
            .add_done_callback(self._spin_done)

    def _spin_done(self, fut):
        self.spinning = False
        res = fut.result().result
        if res and res.total_elapsed_time.sec > 0:
            self.get_logger().info('Spin completado')
        else:
            self.get_logger().warn('Spin fallido')
        self.goal_active = False

# ---------- main ----------
def main(args=None):
    rclpy.init(args=args)
    node = RandomExplorer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
