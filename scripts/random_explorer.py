#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
RandomExplorer mejorado:
• Elige cada objetivo en la celda libre con mayor distancia a obstáculos.
• Detecta atasco: si no avanza 3 s → gira 360° (Spin) y vuelve a planificar.
• Si sigue sin salida tras el giro, da marcha atrás 30 cm (BackUp).
"""
import rclpy
from rclpy.node        import Node
from rclpy.action      import ActionClient
from geometry_msgs.msg import PoseStamped, Point
from nav2_msgs.action  import NavigateToPose, BackUp, Spin
from nav_msgs.msg      import OccupancyGrid, Odometry
from action_msgs.msg   import GoalStatus
import numpy as np
import cv2, random, math


class RandomExplorer(Node):
    # ---------- inicialización ----------
    def __init__(self):
        super().__init__('random_explorer')

        # Action clients Nav2
        self.nav_client    = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.backup_client = ActionClient(self, BackUp,        'back_up')
        self.spin_client   = ActionClient(self, Spin,          'spin')

        # Subscripciones
        self.create_subscription(OccupancyGrid, '/global_costmap/costmap',
                                 self._costmap_cb, 10)
        self.create_subscription(Odometry,        'odom',
                                 self._odom_cb,   20)

        # Buffers
        self.costmap_raw: np.ndarray | None = None
        self.free_cells: list[tuple[int, int]] = []
        self.map_info   = None

        # Estado de navegación
        self.goal_active  = False
        self.nav_goal_hdl = None
        self.backing_up   = False
        self.spinning     = False

        # Detectar atasco mediante odometría
        self.last_pose   = None
        self.last_move_t = self.get_clock().now()
        self.stuck_dist    = 0.02   # 2 cm sin avanzar cuenta como bloqueado
        self.stuck_timeout = 3.0    # …durante 3 s

        # Bucle principal
        self.create_timer(1.0, self._tick)

    # ---------- callbacks ----------
    def _costmap_cb(self, msg: OccupancyGrid):
        self.costmap_raw = np.frombuffer(msg.data, dtype=np.int8)  # 1-D
        data = self.costmap_raw.reshape(msg.info.height, msg.info.width)
        free = np.where(data < 10)
        self.free_cells = list(zip(free[0], free[1]))
        self.map_info   = msg.info

    def _odom_cb(self, msg: Odometry):
        p = msg.pose.pose.position
        if self.last_pose:
            if math.hypot(p.x - self.last_pose.x, p.y - self.last_pose.y) > self.stuck_dist:
                self.last_move_t = self.get_clock().now()
        self.last_pose = p

    # ---------- bucle de control ----------
    def _tick(self):
        if self.backing_up or self.spinning:
            return

        # Atasco: sin progreso el tiempo umbral
        stalled = (self.goal_active and
                   (self.get_clock().now() - self.last_move_t).nanoseconds * 1e-9
                   > self.stuck_timeout)

        if stalled:
            self.get_logger().warn('Atasco detectado → giro 360°')
            self._do_spin()
            return

        if not self.goal_active:
            self._send_new_goal()

    # ---------- navegación ----------
    def _send_new_goal(self):
        if not (self.nav_client.server_is_ready() and self.free_cells):
            return

        # Seleccionar la celda libre con mayor distancia a obstáculos
        r, c = self._pick_farthest_cell()

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

        goal = NavigateToPose.Goal();  goal.pose = pose
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

    # ---------- giro de recuperación ----------
    def _do_spin(self, angle: float = 2 * math.pi):
        if not self.spin_client.server_is_ready():
            self.get_logger().warn('Servidor Spin no listo')
            self._do_backup()          # fallback
            return
        if self.nav_goal_hdl:
            self.nav_goal_hdl.cancel_goal_async()
        goal = Spin.Goal()
        goal.target_yaw = angle
        goal.time_allowance.sec = 10
        self.spinning = True
        self.spin_client.send_goal_async(goal)\
            .add_done_callback(self._spin_done)

    def _spin_done(self, fut):
        self.spinning = False
        result = fut.result().result
        if result and result.total_elapsed_time.sec > 0:
            self.get_logger().info('Spin completado')
        else:
            self.get_logger().warn('Spin fallido → BackUp')
            self._do_backup()
        self.goal_active = False

    # ---------- retroceso ----------
    def _do_backup(self):
        if not self.backup_client.server_is_ready():
            self.get_logger().warn('Servidor BackUp no listo')
            return
        goal = BackUp.Goal()
        goal.target = Point(x=-0.30, y=0.0, z=0.0)   # 30 cm hacia atrás
        goal.speed  = 0.10
        goal.time_allowance.sec = 5
        self.backing_up = True
        self.backup_client.send_goal_async(goal)\
                          .add_done_callback(self._backup_done)

    def _backup_done(self, _):
        self.backing_up  = False
        self.goal_active = False
        self.get_logger().info('BackUp completado')

    # ---------- utilidades ----------
    def _pick_farthest_cell(self) -> tuple[int, int]:
        """Distancia euclídea a obstáculos mediante OpenCV distance transform."""
        if self.costmap_raw is None or not self.free_cells:
            raise RuntimeError('Costmap todavía no recibido')
        grid = self.costmap_raw.reshape(self.map_info.height, self.map_info.width)

        # Ocupado >=50 → 0, libre → 255
        occ = np.where(grid >= 50, 0, 255).astype(np.uint8)
        dist = cv2.distanceTransform(occ, cv2.DIST_L2, 3)  # píxeles

        # Elige la celda libre cuyo valor en 'dist' sea máximo
        return max(self.free_cells, key=lambda rc: dist[rc])

# ---------- main ----------
def main(args=None):
    rclpy.init(args=args)
    node = RandomExplorer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

