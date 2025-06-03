#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
RandomExplorer ‘ágil’:
• Elige la celda libre más lejana DENTRO de ±60° respecto al heading actual.
• Si no avanza ≥10 cm en 8 s → Back-Up 20 cm y nuevo goal (sin Spin).
"""
import rclpy, math, random
import numpy as np, cv2
from rclpy.node        import Node
from rclpy.action      import ActionClient
from geometry_msgs.msg import PoseStamped, Point
from nav_msgs.msg      import OccupancyGrid, Odometry
from nav2_msgs.action  import NavigateToPose, BackUp
from action_msgs.msg   import GoalStatus


class RandomExplorer(Node):
    def __init__(self):
        super().__init__('random_explorer_fast')

        # Action servers Nav2
        self.nav_cli   = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.back_cli  = ActionClient(self, BackUp,        'back_up')

        # Subscripciones
        self.create_subscription(OccupancyGrid, '/global_costmap/costmap',
                                 self._map_cb, 10)
        self.create_subscription(Odometry, 'odom', self._odom_cb, 20)

        # Buffers
        self.dist      = None
        self.free      = []
        self.info      = None
        self.pose_now  = None  # Pose actual (x, y, yaw)

        # Estado
        self.goal_hdl  = None
        self.goal_act  = False
        self.backing   = False

        # Parámetros atasco
        self.stuck_d     = 0.10   # 10 cm
        self.stuck_t     = 8.0    # 8 s
        self.last_move_t = self.get_clock().now()

        # Timer 1 Hz
        self.create_timer(1.0, self._tick)

    # ---------- callbacks ----------
    def _map_cb(self, msg: OccupancyGrid):
        g = np.frombuffer(msg.data, dtype=np.int8).reshape(msg.info.height,
                                                           msg.info.width)
        self.free = list(zip(*np.where(g < 10)))
        self.info = msg.info
        occ       = np.where(g >= 50, 0, 255).astype(np.uint8)
        self.dist = cv2.distanceTransform(occ, cv2.DIST_L2, 3)

    def _odom_cb(self, msg: Odometry):
        p  = msg.pose.pose.position
        ori = msg.pose.pose.orientation
        yaw = math.atan2(2*(ori.w*ori.z + ori.x*ori.y),
                         1 - 2*(ori.y**2 + ori.z**2))
        if self.pose_now:
            dx = p.x - self.pose_now[0];  dy = p.y - self.pose_now[1]
            if math.hypot(dx, dy) > self.stuck_d:
                self.last_move_t = self.get_clock().now()
        self.pose_now = (p.x, p.y, yaw)

    # ---------- bucle principal ----------
    def _tick(self):
        if self.backing:
            return

        # ¿Atasco?
        idle = (self.goal_act and
                (self.get_clock().now() - self.last_move_t).nanoseconds*1e-9
                > self.stuck_t)
        if idle:
            self._do_backup()
            return

        if not self.goal_act:
            self._send_goal()

    # ---------- mando de navegación ----------
    def _send_goal(self):
        if not (self.nav_cli.server_is_ready() and self.free and
                self.dist is not None and self.pose_now):
            return

        # ------- selección orientada -------
        x0, y0, yaw = self.pose_now
        cos_yaw, sin_yaw = math.cos(yaw), math.sin(yaw)

        def score(rc):
            r, c = rc
            x = c*self.info.resolution+self.info.origin.position.x
            y = r*self.info.resolution+self.info.origin.position.y
            dx, dy = x-x0, y-y0
            ang = math.atan2(dy, dx) - yaw
            ang = (ang + math.pi) % (2*math.pi) - math.pi  # [-π, π]
            # Penaliza ángulos >60°
            ang_penalty = 0 if abs(ang) <= math.pi/3 else -999
            return self.dist[r, c] + ang_penalty

        r, c = max(self.free, key=score)

        # ------- genera goal PoseStamped -------
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp    = self.get_clock().now().to_msg()
        pose.pose.position.x = c*self.info.resolution + self.info.origin.position.x \
                               + self.info.resolution/2
        pose.pose.position.y = r*self.info.resolution + self.info.origin.position.y \
                               + self.info.resolution/2
        yaw_goal = random.uniform(-math.pi, math.pi)
        pose.pose.orientation.z = math.sin(yaw_goal/2)
        pose.pose.orientation.w = math.cos(yaw_goal/2)

        g = NavigateToPose.Goal(); g.pose = pose
        self.goal_act = True
        self.nav_cli.send_goal_async(g).add_done_callback(self._goal_resp)

    def _goal_resp(self, fut):
        self.goal_hdl = fut.result()
        if not self.goal_hdl or not self.goal_hdl.accepted:
            self.goal_act = False
            return
        self.goal_hdl.get_result_async().add_done_callback(self._goal_done)

    def _goal_done(self, fut):
        st = fut.result().status
        if st != GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info(f'Goal terminó con status {st}')
        self.goal_act = False

    # ---------- Back-Up ----------
    def _do_backup(self):
        if not self.back_cli.server_is_ready():
            self.get_logger().warn('BackUp no disponible')
            self.goal_act = False
            return
        if self.goal_hdl:
            self.goal_hdl.cancel_goal_async()
        g = BackUp.Goal()
        g.target = Point(x=-0.20, y=0.0, z=0.0)  # 20 cm atrás
        g.speed  = 0.10
        g.time_allowance.sec = 4
        self.backing = True
        self.back_cli.send_goal_async(g).add_done_callback(self._backup_done)

    def _backup_done(self, _):
        self.backing  = False
        self.goal_act = False
        self.last_move_t = self.get_clock().now()  # evita bucle inmediato


def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(RandomExplorer())
    rclpy.shutdown()


if __name__ == '__main__':
    main()
