#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
StraightExplorer
────────────────
1. Toma la orientación actual del robot.
2. Traza un rayo hacia delante (±15°) hasta 3 m buscando la celda libre más lejana
   y la pone como goal.
3. Si en 5 s no avanza ≥8 cm O el goal termina con status distinto de SUCCEEDED:
      – Cancela el goal
      – Back-Up 0.15 m
      – Repite desde 1.
No hay ningún giro voluntario.
"""
import rclpy, math, numpy as np
from rclpy.node        import Node
from rclpy.action      import ActionClient
from geometry_msgs.msg import PoseStamped, Point
from nav_msgs.msg      import OccupancyGrid, Odometry
from nav2_msgs.action  import NavigateToPose, BackUp
from action_msgs.msg   import GoalStatus

# ───────── ajustes rápidos ─────────
RAY_STEP   = 0.20   # m   – paso de muestreo sobre el rayo
RAY_MAX    = 3.00   # m   – distancia máxima que se intenta
RAY_ANGLE  = math.radians(15)  # ±15° de tolerancia
STUCK_D    = 0.08   # 8 cm sin avance…
STUCK_T    = 5.0    # …durante 5 s → Back-Up
BACK_DIST  = 0.15   # 15 cm atrás

class StraightExplorer(Node):
    def __init__(self):
        super().__init__('straight_explorer')
        self.nav  = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.back = ActionClient(self, BackUp,        'back_up')
        self.create_subscription(OccupancyGrid, '/global_costmap/costmap',
                                 self._map_cb, 10)
        self.create_subscription(Odometry, '/odometry/filtered',
                                 self._odom_cb, 20)

        self.grid = None;  self.info = None
        self.pose = None   # (x, y, yaw)
        self.goal_act = False;  self.backing = False
        self.last_move = self.get_clock().now()

        self.create_timer(1.0, self._tick)

    # ───────── callbacks ─────────
    def _map_cb(self, msg: OccupancyGrid):
        self.grid = np.frombuffer(msg.data, dtype=np.int8)\
                     .reshape(msg.info.height, msg.info.width)
        self.info = msg.info

    def _odom_cb(self, msg: Odometry):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        yaw = math.atan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y*q.y + q.z*q.z))
        if self.pose and math.hypot(p.x - self.pose[0], p.y - self.pose[1]) > STUCK_D:
            self.last_move = self.get_clock().now()
        self.pose = (p.x, p.y, yaw)

    # ───────── bucle principal ─────────
    def _tick(self):
        if self.backing:
            return

        # atasco: sin progreso suficiente
        if ( self.goal_act and
             (self.get_clock().now() - self.last_move).nanoseconds * 1e-9 > STUCK_T ):
            self._cancel_goal_and_backup()
            return

        if not self.goal_act:
            self._send_straight_goal()

    # ───────── generación de goal recto ─────────
    def _send_straight_goal(self):
        if not (self.nav.server_is_ready() and self.grid is not None and self.pose):
            return
        x0, y0, yaw = self.pose
        best = None
        for ang in [0,  RAY_ANGLE, -RAY_ANGLE]:               # 0°, +15°, −15°
            ca, sa = math.cos(yaw+ang), math.sin(yaw+ang)
            for d in np.arange(RAY_STEP, RAY_MAX+RAY_STEP, RAY_STEP):
                x = x0 + ca * d
                y = y0 + sa * d
                r = int((y - self.info.origin.position.y) / self.info.resolution)
                c = int((x - self.info.origin.position.x) / self.info.resolution)
                if 0 <= r < self.info.height and 0 <= c < self.info.width:
                    if self.grid[r, c] < 10:   # libre
                        best = (x, y)           # sigue buscando más lejos
                    else:
                        break                  # chocó con obstáculo en este rayo
        if not best:
            self.get_logger().warn('Sin camino libre adelante → reintento 1 s')
            return

        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x, pose.pose.position.y = best
        pose.pose.orientation.w = 1.0
        g = NavigateToPose.Goal();  g.pose = pose
        self.goal_act = True
        self.nav.send_goal_async(g).add_done_callback(self._goal_resp)

    def _goal_resp(self, fut):
        h = fut.result()
        if not h or not h.accepted:
            self.goal_act = False;  return
        h.get_result_async().add_done_callback(self._goal_done)

    def _goal_done(self, fut):
        st = fut.result().status
        self.goal_act = False
        if st != GoalStatus.STATUS_SUCCEEDED:          # chocó o abortó
            self._cancel_goal_and_backup()

    # ───────── Back-Up y retry ─────────
    def _cancel_goal_and_backup(self):
        if self.backing:
            return
        if self.nav.server_is_ready():
            self.nav.cancel_all_goals()   # cancela cualquier goal activo
        if not self.back.server_is_ready():
            self.get_logger().warn('BackUp no disponible');  return
        g = BackUp.Goal()
        g.target = Point(x=-BACK_DIST, y=0.0, z=0.0)
        g.speed  = 0.10
        g.time_allowance.sec = 3
        self.backing = True
        self.back.send_goal_async(g).add_done_callback(self._backup_done)

    def _backup_done(self, _):
        self.backing = False
        self.last_move = self.get_clock().now()  # evita falso atasco inmediato

# ─────────────────────────────────────────────
def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(StraightExplorer())
    rclpy.shutdown()
if __name__ == '__main__':
    main()
