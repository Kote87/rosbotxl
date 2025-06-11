#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Frontier120Explorer
Explora todo el mapa usando frontiers, con haz frontal ±60°.
 - Usa Nav2 (NavigateToPose) para ir a cada frontier.
 - Gira 60° pasos hasta encontrar frontiers en el haz.
 - No publica BackUp / Spin; giros los hace vía /cmd_vel.
 - Cuando no quedan frontiers tras un giro completo, se queda
   rotando lentamente (modo 'stand‑by de explorador').
"""

import rclpy, math, numpy as np, cv2
from rclpy.node  import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped, Quaternion, Twist
from nav_msgs.msg      import OccupancyGrid, Odometry
from nav2_msgs.action  import NavigateToPose
from action_msgs.msg   import GoalStatus

# ---------- Parámetros ---------------------------------------------
FOV_DEG         = 120                # campo frontal total (±60°)
FOV_RAD         = math.radians(FOV_DEG/2)   # 60° en rad
MIN_FRONTIER_CLEARANCE = 0.25        # m – distancia mínima a obsts
MAP_FREE_THR    = 10                 # grid <10 => libre
ROT_STEP_DEG    = 60                 # giro incremental cuando no hay frontier
ROT_SPEED       = 0.5                # rad/s (cmd_vel)
GOAL_RETRY_SEC  = 3.0                # espera entre goals fallidos

# ---------- Ayudas --------------------------------------------------
def yaw_from_quat(q):
    return math.atan2(2*(q.w*q.z + q.x*q.y),
                      1 - 2*(q.y*q.y + q.z*q.z))

def quat_from_yaw(y):
    q = Quaternion()
    q.z = math.sin(y/2)
    q.w = math.cos(y/2)
    return q

# ---------- Nodo principal -----------------------------------------
class Frontier120Explorer(Node):
    def __init__(self):
        super().__init__('frontier120_explorer')

        # Nav2 NavigateToPose
        self.nav_cli = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # cmd_vel p/ giros manuales
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Subscripciones
        self.create_subscription(OccupancyGrid, '/global_costmap/costmap',
                                 self.map_cb, 10)
        self.create_subscription(Odometry, '/odometry/filtered',
                                 self.odom_cb, 20)

        # Buffers
        self.grid = None         # np.int8 OccupancyGrid
        self.info = None         # Metadata del mapa
        self.pose = None         # (x,y,yaw)
        self.dist = None         # distanceTransform
        self.frontiers = []      # lista de (r,c)
        self.goal_active = False

        # Giro incremental
        self.rotating      = False
        self.rot_start_sec = 0.0
        self.rot_target    = 0.0      # rad
        self.yaw_start     = 0.0

        # Timer principal 1 Hz
        self.create_timer(1.0, self.tick)

    # ---------------- Costmap ---------------------------------------
    def map_cb(self, msg: OccupancyGrid):
        self.info = msg.info
        h, w = msg.info.height, msg.info.width
        grid = np.frombuffer(msg.data, dtype=np.int8).reshape(h, w)

        # Si conozco mi pose → ignoro obstáculos detrás
        if self.pose:
            x_r, y_r, yaw_r = self.pose
            res = self.info.resolution
            for r in range(h):
                for c in range(w):
                    if grid[r, c] >= 50:           # obstáculo
                        wx = c*res + self.info.origin.position.x + res/2
                        wy = r*res + self.info.origin.position.y + res/2
                        dx, dy = wx - x_r, wy - y_r
                        ang = math.atan2(dy, dx) - yaw_r
                        ang = (ang + math.pi) % (2*math.pi) - math.pi
                        if abs(ang) > math.pi/2:   # detrás
                            grid[r, c] = 0         # se vuelve libre

        self.grid = grid
        self.dist = cv2.distanceTransform(
            np.where(grid >= 50, 0, 255).astype(np.uint8),
            cv2.DIST_L2, 3)

        # ---------- detectar frontiers ----------
        self.frontiers = []
        if self.pose is None:
            return
        x_r, y_r, yaw_r = self.pose
        res = self.info.resolution
        for r in range(1, h-1):
            for c in range(1, w-1):
                if grid[r, c] < MAP_FREE_THR:
                    nbr = grid[r-1:r+2, c-1:c+2]
                    if np.any(nbr == -1):          # vecino desconocido
                        # coordenadas mundo
                        wx = c*res + self.info.origin.position.x + res/2
                        wy = r*res + self.info.origin.position.y + res/2
                        dx, dy = wx - x_r, wy - y_r
                        ang = math.atan2(dy, dx) - yaw_r
                        ang = (ang + math.pi) % (2*math.pi) - math.pi
                        if abs(ang) <= FOV_RAD:    # dentro ±60°
                            # distancia en metros a obsts
                            d_clear = self.dist[r, c]*res
                            if d_clear >= MIN_FRONTIER_CLEARANCE:
                                self.frontiers.append((r, c))

    # ---------------- Odom ------------------------------------------
    def odom_cb(self, msg: Odometry):
        p = msg.pose.pose.position
        y = yaw_from_quat(msg.pose.pose.orientation)
        self.pose = (p.x, p.y, y)

    # ---------------- Timer principal --------------------------------
    def tick(self):
        # 1) Si estamos girando manualmente
        if self.rotating:
            self.update_rotation()
            return

        # 2) Si hay goal activo → nada
        if self.goal_active:
            return

        # 3) ¿Hay frontiers en el haz?
        if self.frontiers:
            self.send_frontier_goal()
        else:
            # No hay frontiers → giramos 60°
            self.start_rotation( math.radians(ROT_STEP_DEG) )

    # ---------------- Giros manuales ---------------------------------
    def start_rotation(self, delta_yaw):
        if self.pose is None:
            return
        self.rotating      = True
        self.rot_start_sec = self.get_clock().now().seconds_nanoseconds()[0]
        self.yaw_start     = self.pose[2]
        self.rot_target    = self.pose[2] + delta_yaw
        # normalizar
        self.rot_target = (self.rot_target + math.pi) % (2*math.pi) - math.pi
        # publica cmd_vel ang.z
        twist = Twist()
        twist.angular.z = ROT_SPEED if delta_yaw > 0 else -ROT_SPEED
        self.cmd_pub.publish(twist)

    def update_rotation(self):
        now_sec = self.get_clock().now().seconds_nanoseconds()[0]
        if self.pose is None:
            return
        yaw = self.pose[2]
        # ¿hemos alcanzado el ángulo objetivo?
        err = (self.rot_target - yaw + math.pi) % (2*math.pi) - math.pi
        if abs(err) < 0.05:           # ~3 °
            # parar
            self.cmd_pub.publish(Twist())  # cero
            self.rotating = False
            return
        # sigue publicando velocidad
        twist = Twist()
        twist.angular.z = ROT_SPEED if err > 0 else -ROT_SPEED
        self.cmd_pub.publish(twist)

    # ---------------- Enviar goal Nav2 --------------------------------
    def send_frontier_goal(self):
        if not self.nav_cli.server_is_ready():
            return
        # Elegimos la frontier *más cercana* (dist Euclídea)
        x_r, y_r, _ = self.pose
        res = self.info.resolution
        tgt = min(
            self.frontiers,
            key=lambda rc: math.hypot(
                (rc[1]*res + self.info.origin.position.x + res/2) - x_r,
                (rc[0]*res + self.info.origin.position.y + res/2) - y_r))
        r, c = tgt
        gx = c*res + self.info.origin.position.x + res/2
        gy = r*res + self.info.origin.position.y + res/2

        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp    = self.get_clock().now().to_msg()
        pose.pose.position.x = gx
        pose.pose.position.y = gy
        pose.pose.orientation = quat_from_yaw(self.pose[2])

        goal = NavigateToPose.Goal(); goal.pose = pose
        self.goal_active = True
        fut = self.nav_cli.send_goal_async(goal)
        fut.add_done_callback(self.goal_resp)

    def goal_resp(self, fut):
        h = fut.result()
        if not h or not h.accepted:
            self.goal_active = False
            return
        h.get_result_async().add_done_callback(self.goal_done)

    def goal_done(self, fut):
        st = fut.result().status
        if st != GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info(f'Goal ended with status {st}')
        # dejamos un breve cool‑down para no saturar si falla en bucle
        self.create_timer(GOAL_RETRY_SEC, lambda: None)
        self.goal_active = False

# ---------- main ----------------------------------------------------
def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(Frontier120Explorer())
    rclpy.shutdown()

if __name__ == '__main__':
    main()
