#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
RandomExplorer – con reactividad y FILTRO 180° TRASERO,
SIN usar las acciones BackUp y Spin de Nav2.
En su lugar, publica cmd_vel directamente para retroceder y girar.
"""

import rclpy, math, numpy as np, cv2
from rclpy.node        import Node
from rclpy.action      import ActionClient
from geometry_msgs.msg import PoseStamped, Point, Quaternion, Twist
from nav_msgs.msg      import OccupancyGrid, Odometry
from nav2_msgs.action  import NavigateToPose
from action_msgs.msg   import GoalStatus

STUCK_DIST   = 0.08
STUCK_TIME   = 3.0
BACK_DIST    = 0.20
SPIN_YAW     = math.pi  # 180°
COOLDOWN     = 10.0

MIN_DIST_FROM_OBST = 0.3
MAX_ANG            = math.radians(60)
CLOSE_OBST_DIST_M  = 0.3  # obstáculo cercano => back-up inmediato

def quaternion_from_yaw(yaw: float) -> Quaternion:
    q = Quaternion()
    q.z = math.sin(yaw / 2.0)
    q.w = math.cos(yaw / 2.0)
    return q

# Estados de REACCION manual
IDLE       = 0
BACKING    = 1
SPINNING   = 2

class RandomExplorer(Node):
    def __init__(self):
        super().__init__('random_explorer')

        # Accion principal de NavigateToPose (Nav2)
        self.nav_cli  = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # Publicador cmd_vel para movernos manualmente
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Subscripciones
        self.create_subscription(OccupancyGrid, '/global_costmap/costmap',
                                 self._map_cb, 10)
        self.create_subscription(Odometry, '/odometry/filtered',
                                 self._odom_cb, 20)

        # Buffers
        self.grid = None
        self.info = None
        self.dist = None
        self.free = []
        self.pose = None  # (x, y, yaw)

        self.goal_act   = False
        self.last_move  = self.get_clock().now()
        self.last_react = self.get_clock().now()

        # Estado de reacciones manuales
        self.react_state  = IDLE
        self.react_start  = None  # tiempo de inicio de la maniobra
        self.back_done    = False # para saber si ya se terminó de retroceder

        # Timer principal a 2 Hz
        self.create_timer(0.5, self._tick)

    # ----------------------------------------------------------------------
    # CALLBACKS
    # ----------------------------------------------------------------------
    def _map_cb(self, msg: OccupancyGrid):
        self.info = msg.info
        self.grid = np.frombuffer(msg.data, dtype=np.int8).reshape(msg.info.height, msg.info.width)

        # ---- Filtrar obstáculos detrás (±90°) ----
        if self.pose:
            x_r, y_r, yaw_r = self.pose
            h, w = self.grid.shape
            for r in range(h):
                for c in range(w):
                    if self.grid[r, c] >= 50:
                        wx = c*self.info.resolution + self.info.origin.position.x + self.info.resolution/2
                        wy = r*self.info.resolution + self.info.origin.position.y + self.info.resolution/2
                        dx = wx - x_r
                        dy = wy - y_r
                        angle = math.atan2(dy, dx) - yaw_r
                        while angle > math.pi:   angle -= 2*math.pi
                        while angle < -math.pi:  angle += 2*math.pi
                        if abs(angle) > math.pi/2:
                            self.grid[r, c] = 0  # ignora obstáculo detrás

        self.free = list(zip(*np.where(self.grid < 10)))
        bin_map = np.where(self.grid >= 50, 0, 255).astype(np.uint8)
        self.dist = cv2.distanceTransform(bin_map, cv2.DIST_L2, 3)

    def _odom_cb(self, msg: Odometry):
        p  = msg.pose.pose.position
        q  = msg.pose.pose.orientation
        yaw = math.atan2(2*(q.w*q.z + q.x*q.y),
                         1 - 2*(q.y*q.y + q.z*q.z))

        if self.pose and math.hypot(p.x - self.pose[0], p.y - self.pose[1]) > STUCK_DIST:
            self.last_move = self.get_clock().now()

        self.pose = (p.x, p.y, yaw)

    # ----------------------------------------------------------------------
    # BUCLE PRINCIPAL
    # ----------------------------------------------------------------------
    def _tick(self):
        now = self.get_clock().now()

        # 1) Si estamos haciendo maniobra manual (BACKING o SPINNING), ejecutar
        if self.react_state != IDLE:
            self._exec_reaction(now)
            return

        # 2) Chequear si atascado o close_obstacle
        time_since_move  = (now - self.last_move).nanoseconds * 1e-9
        time_since_react = (now - self.last_react).nanoseconds * 1e-9

        stuck = ( self.goal_act and
                  (time_since_move > STUCK_TIME) and
                  (time_since_react > COOLDOWN) )

        close_obstacle = self._check_close_obstacle()

        if stuck or close_obstacle:
            self.get_logger().info(f'Activando reacción: stuck={stuck} close_obstacle={close_obstacle}')
            self.last_react = now
            self._react()  # iniciamos maniobra
            return

        # 3) Si no hay goal activo, enviamos uno
        if not self.goal_act:
            self._send_goal()

    def _check_close_obstacle(self) -> bool:
        if self.grid is None or self.pose is None:
            return False

        x_r, y_r, _ = self.pose
        cx = int((x_r - self.info.origin.position.x) / self.info.resolution)
        cy = int((y_r - self.info.origin.position.y) / self.info.resolution)

        if cx<0 or cy<0 or cx>=self.info.width or cy>=self.info.height:
            return False

        cells_radius = int(CLOSE_OBST_DIST_M / self.info.resolution)
        rmin = max(0, cy - cells_radius)
        rmax = min(self.info.height-1, cy + cells_radius)
        cmin = max(0, cx - cells_radius)
        cmax = min(self.info.width-1,  cx + cells_radius)

        submap = self.grid[rmin:rmax+1, cmin:cmax+1]
        return np.any(submap >= 50)

    # ----------------------------------------------------------------------
    # ELECCIÓN DE OBJETIVO
    # ----------------------------------------------------------------------
    def _send_goal(self):
        if not (self.nav_cli.server_is_ready() and self.free and
                self.dist is not None and self.pose):
            return

        x0, y0, yaw0 = self.pose
        res = self.info.resolution

        # filtrar celdas con dist >= MIN_DIST_FROM_OBST
        safe_cells = []
        for (r, c) in self.free:
            dist_m = self.dist[r, c] * res
            if dist_m >= MIN_DIST_FROM_OBST:
                safe_cells.append((r,c))

        if not safe_cells:
            self.get_logger().warn('No hay celdas seguras (dist > {:.2f} m).'.format(MIN_DIST_FROM_OBST))
            return

        front_cells = []
        for (r,c) in safe_cells:
            wx = c*res + self.info.origin.position.x + res/2
            wy = r*res + self.info.origin.position.y + res/2
            dx = wx - x0
            dy = wy - y0
            angle = math.atan2(dy, dx) - yaw0
            while angle > math.pi:   angle -= 2*math.pi
            while angle < -math.pi:  angle += 2*math.pi
            if abs(angle) <= MAX_ANG:
                front_cells.append((r,c))

        if not front_cells:
            self.get_logger().warn('No hay celdas en el frente que cumplan ±{:.1f}°.'.format(math.degrees(MAX_ANG)))
            return

        r, c = max(front_cells, key=lambda rc: self.dist[rc])
        goal_x = c*res + self.info.origin.position.x + res/2
        goal_y = r*res + self.info.origin.position.y + res/2

        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = goal_x
        pose.pose.position.y = goal_y

        q = quaternion_from_yaw(yaw0)
        pose.pose.orientation = q

        goal = NavigateToPose.Goal()
        goal.pose = pose

        self.goal_act = True
        self.nav_cli.send_goal_async(goal).add_done_callback(self._goal_resp)

    def _goal_resp(self, fut):
        h = fut.result()
        if not h or not h.accepted:
            self.goal_act = False
            return
        h.get_result_async().add_done_callback(self._goal_done)

    def _goal_done(self, fut):
        if fut.result().status != GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info(f'Goal end status {fut.result().status}')
        self.goal_act = False

    # ----------------------------------------------------------------------
    # REACCIÓN: Back-Up + Spin manual
    # ----------------------------------------------------------------------
    def _react(self):
        """
        Inicia la reacción manual con cmd_vel:
         1) Retroceder un tiempo hasta cubrir BACK_DIST.
         2) Girar en el sitio 180°.
        Se maneja en _exec_reaction.
        """
        # Cancelamos goal
        if self.goal_act:
            self.nav_cli.cancel_all_goals()
            self.goal_act = False

        self.react_state = BACKING
        self.react_start = self.get_clock().now()
        self.back_done   = False

    def _exec_reaction(self, now):
        """
        Ejecuta la maniobra de back-up + spin en función del tiempo transcurrido.
        """
        dt = (now - self.react_start).nanoseconds * 1e-9

        # 1) BACKING => publicamos cmd_vel lineal negativo hasta cubrir BACK_DIST
        if self.react_state == BACKING and not self.back_done:
            # Estimamos si ya hemos retrocedido BACK_DIST
            # En lugar de odometría exacta, hacemos un tiempo fijo
            # supongamos que retrocedemos a 0.1 m/s => tardamos (BACK_DIST / 0.1) s
            back_time = BACK_DIST / 0.1  # con speed=0.1 m/s
            if dt < back_time:
                twist = Twist()
                twist.linear.x = -0.1  # retroceder
                self.cmd_vel_pub.publish(twist)
            else:
                # hemos acabado retroceso
                self.back_done = True
                self.react_start = now  # reiniciamos tiempo
                # paramos
                stop_msg = Twist()
                self.cmd_vel_pub.publish(stop_msg)

        # 2) Si ya acabamos retro, pasamos a SPINNING => girar 180° a 0.4 rad/s => tardamos ~7.85 s
        if self.react_state == BACKING and self.back_done:
            self.react_state = SPINNING
            self.react_start = now
            return

        if self.react_state == SPINNING:
            spin_time = abs(SPIN_YAW) / 0.4  # si giramos a 0.4 rad/s
            if dt < spin_time:
                twist = Twist()
                twist.angular.z = 0.4 if SPIN_YAW >= 0 else -0.4
                self.cmd_vel_pub.publish(twist)
            else:
                # terminar spin
                stop_msg = Twist()
                self.cmd_vel_pub.publish(stop_msg)
                # volver a estado IDLE
                self.react_state = IDLE

# --------------------------------------------------------------------------
def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(RandomExplorer())
    rclpy.shutdown()

if __name__ == '__main__':
    main()
