#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
RandomExplorer – versión “central + reacción”
• Objetivo: celda más lejana de obstáculos (distance-transform).
• Atasco = 8 cm sin progreso durante 5 s → Back-Up 0.2 m + Spin 180 °.
"""

import rclpy, math, numpy as np, cv2
from rclpy.node        import Node
from rclpy.action      import ActionClient
from geometry_msgs.msg import PoseStamped, Point
from nav_msgs.msg      import OccupancyGrid, Odometry
from nav2_msgs.action  import NavigateToPose, BackUp, Spin
from action_msgs.msg   import GoalStatus

# ---------- parámetros fáciles de afinar ----------
STUCK_DIST   = 0.08   # 8 cm sin moverse…
STUCK_TIME   = 5.0    # …durante 5 s ⇒ reacción
BACK_DIST    = 0.20   # retroceso 20 cm
SPIN_YAW     = math.pi    # 180 °
COOLDOWN     = 15.0   # mínimo 15 s entre reacciones

class RandomExplorer(Node):
    def __init__(self):
        super().__init__('random_explorer')

        # ── action clients Nav2 ────────────────────────────────
        self.nav_cli  = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.back_cli = ActionClient(self, BackUp,        'back_up')
        self.spin_cli = ActionClient(self, Spin,          'spin')

        # ── subscripciones ────────────────────────────────────
        self.create_subscription(OccupancyGrid, '/global_costmap/costmap',
                                 self._map_cb, 10)
        self.create_subscription(Odometry, '/odometry/filtered',
                                 self._odom_cb, 20)

        # ── buffers/estado ────────────────────────────────────
        self.grid = None;  self.info = None
        self.dist = None   # distance-transform
        self.free = []     # celdas libres
        self.pose = None   # (x, y, yaw)

        self.goal_act = False
        self.backing  = False
        self.spinning = False
        self.last_move = self.get_clock().now()
        self.last_react = self.get_clock().now()

        # ── timer principal 1 Hz ──────────────────────────────
        self.create_timer(1.0, self._tick)

    # ---------- callbacks --------------------------------------------------
    def _map_cb(self, msg: OccupancyGrid):
        self.info = msg.info
        self.grid = np.frombuffer(msg.data, dtype=np.int8) \
                     .reshape(msg.info.height, msg.info.width)
        self.free = list(zip(*np.where(self.grid < 10)))
        self.dist = cv2.distanceTransform(
                        np.where(self.grid >= 50, 0, 255).astype(np.uint8),
                        cv2.DIST_L2, 3)

    def _odom_cb(self, msg: Odometry):
        p  = msg.pose.pose.position
        q  = msg.pose.pose.orientation
        yaw = math.atan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y*q.y + q.z*q.z))
        if self.pose and math.hypot(p.x - self.pose[0], p.y - self.pose[1]) > STUCK_DIST:
            self.last_move = self.get_clock().now()
        self.pose = (p.x, p.y, yaw)

    # ---------- bucle principal -------------------------------------------
    def _tick(self):
        # si está ejecutando una reacción, esperar
        if self.backing or self.spinning:
            return

        # detectar atasco
        now = self.get_clock().now()
        stuck = ( self.goal_act and
                  (now - self.last_move).nanoseconds*1e-9 > STUCK_TIME and
                  (now - self.last_react).nanoseconds*1e-9 > COOLDOWN )

        if stuck:
            self.last_react = now
            self._react()
            return

        if not self.goal_act:
            self._send_goal()

    # ---------- elección de objetivo --------------------------------------
    def _send_goal(self):
        if not (self.nav_cli.server_is_ready() and self.free and
                self.dist is not None and self.pose):
            return

        # celda libre con mayor distancia a obstáculos
        r, c = max(self.free, key=lambda rc: self.dist[rc])

        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp    = self.get_clock().now().to_msg()
        pose.pose.position.x = c*self.info.resolution + self.info.origin.position.x \
                               + self.info.resolution/2
        pose.pose.position.y = r*self.info.resolution + self.info.origin.position.y \
                               + self.info.resolution/2
        pose.pose.orientation.w = 1.0   # orientación la ajusta Nav2

        goal = NavigateToPose.Goal();  goal.pose = pose
        self.goal_act = True
        self.nav_cli.send_goal_async(goal)\
                    .add_done_callback(self._goal_resp)

    def _goal_resp(self, fut):
        h = fut.result()
        if not h or not h.accepted:
            self.goal_act = False;  return
        h.get_result_async().add_done_callback(self._goal_done)

    def _goal_done(self, fut):
        if fut.result().status != GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info(f'Goal end status {fut.result().status}')
        self.goal_act = False

    # ---------- reacción: Back-Up + Spin ----------------------------------
    def _react(self):
        if not self.back_cli.server_is_ready():
            self.get_logger().warn('BackUp no disponible');  return
        if self.goal_act:
            self.nav_cli.cancel_all_goals()
            self.goal_act = False

        # retroceso
        g = BackUp.Goal()
        g.target = Point(x=-BACK_DIST, y=0.0, z=0.0)
        g.speed  = 0.10
        g.time_allowance.sec = 4
        self.backing = True
        self.back_cli.send_goal_async(g).add_done_callback(self._back_done)

    def _back_done(self, _):
        self.backing = False
        # giro 180 °
        if not self.spin_cli.server_is_ready():
            return
        s = Spin.Goal()
        s.target_yaw = SPIN_YAW
        s.time_allowance.sec = 6
        self.spinning = True
        self.spin_cli.send_goal_async(s).add_done_callback(self._spin_done)

    def _spin_done(self, _):
        self.spinning = False
        # listo para un nuevo goal en el próximo tick

# ---------- main ----------------------------------------------------------
def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(RandomExplorer())
    rclpy.shutdown()

if __name__ == '__main__':
    main()
