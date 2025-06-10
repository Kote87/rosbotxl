#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
RandomExplorer – versión con ajustes de reactividad y filtrado de 180° traseros.
 - Se ignoran obstáculos detrás del robot (±90°).
 - Se aumenta frecuencia de tick (2 Hz).
 - Se reduce STUCK_TIME (3s) y COOLDOWN (10s).
 - Chequeo adicional de obstáculos cercanos para hacer back-up inmediato.
"""

import rclpy, math, numpy as np, cv2
from rclpy.node        import Node
from rclpy.action      import ActionClient
from geometry_msgs.msg import PoseStamped, Point, Quaternion
from nav_msgs.msg      import OccupancyGrid, Odometry
from nav2_msgs.action  import NavigateToPose, BackUp, Spin
from action_msgs.msg   import GoalStatus

# ---------- parámetros fáciles de afinar ----------
STUCK_DIST   = 0.08   # 8 cm sin moverse…
STUCK_TIME   = 3.0    # reacciona a los 3 s sin moverse (antes eran 5 s)
BACK_DIST    = 0.20   # retroceso 20 cm
SPIN_YAW     = math.pi  # 180° (puedes cambiarlo)
COOLDOWN     = 10.0   # espera al menos 10 s entre reacciones (antes 15 s)

# ---------- parámetros de filtración de celdas ----------
MIN_DIST_FROM_OBST = 0.3    # distancia mínima (m) a obstáculos
MAX_ANG            = math.radians(60)  # filtrar celdas en ±60° frente al robot

# ---------- umbral para "obstáculo inmediato" ----------
CLOSE_OBST_DIST_M  = 0.3   # si detectamos algo a menos de 0.3 m, back-up inmediato

def quaternion_from_yaw(yaw: float) -> Quaternion:
    """
    Crea un cuaternión a partir del yaw (en radianes).
    """
    q = Quaternion()
    q.z = math.sin(yaw / 2.0)
    q.w = math.cos(yaw / 2.0)
    return q

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
        self.grid = None
        self.info = None
        self.dist = None
        self.free = []
        self.pose = None  # (x, y, yaw)

        self.goal_act   = False  # nav goal activo
        self.backing    = False
        self.spinning   = False
        self.last_move  = self.get_clock().now()
        self.last_react = self.get_clock().now()

        # ── timer principal a 2 Hz (0.5 s) ─────────────────────
        self.create_timer(0.5, self._tick)  # frecuencia de 2 Hz

    # ----------------------------------------------------------------------
    #                      CALLBACKS
    # ----------------------------------------------------------------------

    def _map_cb(self, msg: OccupancyGrid):
        """
        Callback que recibe el costmap global.
        1. Convertimos a numpy.
        2. Si tenemos la pose, forzamos que todo lo que está detrás del robot sea libre.
        3. Calculamos distance transform y guardamos la lista de celdas libres.
        """
        self.info = msg.info
        # Convertir a array numpy
        self.grid = np.frombuffer(msg.data, dtype=np.int8).reshape(msg.info.height, msg.info.width)

        # -------------- Filtrar obstáculos detrás del robot -------------
        if self.pose:
            x_r, y_r, yaw_r = self.pose
            h, w = self.grid.shape

            for r in range(h):
                for c in range(w):
                    # Si es obstáculo (>= 50)
                    if self.grid[r, c] >= 50:
                        # Obtener coords en el marco global
                        wx = c*self.info.resolution + self.info.origin.position.x + self.info.resolution/2
                        wy = r*self.info.resolution + self.info.origin.position.y + self.info.resolution/2

                        # Vector relativo al robot
                        dx = wx - x_r
                        dy = wy - y_r

                        # Ángulo relativo al yaw del robot
                        angle = math.atan2(dy, dx) - yaw_r
                        # normalizar
                        while angle > math.pi:
                            angle -= 2*math.pi
                        while angle < -math.pi:
                            angle += 2*math.pi

                        # Si está detrás (fuera de ±90°), lo borramos:
                        if abs(angle) > math.pi/2:
                            self.grid[r, c] = 0  # marcar como libre

        # -------------- Actualizar lista de celdas libres -------------
        self.free = list(zip(*np.where(self.grid < 10)))

        # -------------- Distance transform para buscar zonas alejadas -------------
        bin_map = np.where(self.grid >= 50, 0, 255).astype(np.uint8)
        self.dist = cv2.distanceTransform(bin_map, cv2.DIST_L2, 3)

    def _odom_cb(self, msg: Odometry):
        """
        Callback de odometría: guardamos pose (x, y, yaw).
        Si el robot se mueve más de STUCK_DIST, actualizamos last_move.
        """
        p  = msg.pose.pose.position
        q  = msg.pose.pose.orientation
        yaw = math.atan2(2*(q.w*q.z + q.x*q.y),
                         1 - 2*(q.y*q.y + q.z*q.z))

        if self.pose and math.hypot(p.x - self.pose[0], p.y - self.pose[1]) > STUCK_DIST:
            self.last_move = self.get_clock().now()

        self.pose = (p.x, p.y, yaw)

    # ----------------------------------------------------------------------
    #                      BUCLE PRINCIPAL
    # ----------------------------------------------------------------------

    def _tick(self):
        """
        Se llama 2 veces por segundo.
        - Si estamos haciendo back-up o spin, no hacemos nada.
        - Comprobamos si está atascado (stuck) o hay obstáculo muy cercano.
        - Si sí, reaccionamos (back-up + spin).
        - Si no hay objetivo activo, pedimos uno nuevo.
        """
        if self.backing or self.spinning:
            return

        now = self.get_clock().now()
        time_since_move  = (now - self.last_move).nanoseconds * 1e-9
        time_since_react = (now - self.last_react).nanoseconds * 1e-9

        # ¿Está atascado?
        stuck = ( self.goal_act and
                  (time_since_move > STUCK_TIME) and
                  (time_since_react > COOLDOWN) )

        # ¿Hay un obstáculo muy cercano delante?
        close_obstacle = self._check_close_obstacle()

        if stuck or close_obstacle:
            self.get_logger().info('Activando reacción: stuck={} close_obstacle={}'.format(stuck, close_obstacle))
            self.last_react = now
            self._react()
            return

        # Si no hay goal, enviamos uno nuevo
        if not self.goal_act:
            self._send_goal()

    def _check_close_obstacle(self) -> bool:
        """
        Revisa si hay un obstáculo en el costmap a menos de CLOSE_OBST_DIST_M.
        """
        if self.grid is None or self.pose is None:
            return False

        x_r, y_r, _ = self.pose
        # Transformar (x_r, y_r) a celdas
        cx = int((x_r - self.info.origin.position.x) / self.info.resolution)
        cy = int((y_r - self.info.origin.position.y) / self.info.resolution)

        # Si está fuera del costmap, devolvemos False
        if cx<0 or cy<0 or cx>=self.info.width or cy>=self.info.height:
            return False

        # Número de celdas en 0.3 m
        cells_radius = int(CLOSE_OBST_DIST_M / self.info.resolution)

        rmin = max(0, cy - cells_radius)
        rmax = min(self.info.height-1, cy + cells_radius)
        cmin = max(0, cx - cells_radius)
        cmax = min(self.info.width-1,  cx + cells_radius)

        # submap = parte del grid alrededor del robot
        submap = self.grid[rmin:rmax+1, cmin:cmax+1]

        # Si cualquier celda en ese vecindario es >= 50, hay obstáculo letal
        if np.any(submap >= 50):
            return True
        return False

    # ----------------------------------------------------------------------
    #                      ELECCIÓN DE OBJETIVO
    # ----------------------------------------------------------------------

    def _send_goal(self):
        """
        Elige la celda más alejada (dist transform) dentro de ±60° (MAX_ANG) y
        al menos a 0.3 m de obstáculos. Envía un NavigateToPose a Nav2.
        """
        if not (self.nav_cli.server_is_ready() and self.free and
                self.dist is not None and self.pose):
            return

        x0, y0, yaw0 = self.pose
        res = self.info.resolution

        # Filtrar celdas por MIN_DIST_FROM_OBST
        safe_cells = []
        for (r, c) in self.free:
            dist_m = self.dist[r, c] * res
            if dist_m >= MIN_DIST_FROM_OBST:
                safe_cells.append((r, c))

        if not safe_cells:
            self.get_logger().warn('No hay celdas seguras (dist > {:.2f} m).'.format(MIN_DIST_FROM_OBST))
            return

        # Filtrar por ángulo ±MAX_ANG con respecto al frente del robot
        front_cells = []
        for (r, c) in safe_cells:
            wx = c*res + self.info.origin.position.x + res/2
            wy = r*res + self.info.origin.position.y + res/2

            dx = wx - x0
            dy = wy - y0
            angle = math.atan2(dy, dx) - yaw0

            # normalizar a -π..π
            while angle > math.pi:
                angle -= 2*math.pi
            while angle < -math.pi:
                angle += 2*math.pi

            if abs(angle) <= MAX_ANG:
                front_cells.append((r, c))

        if not front_cells:
            self.get_logger().warn('No hay celdas en el frente que cumplan ±{:.1f}°.'.format(math.degrees(MAX_ANG)))
            return

        # Tomar la celda con mayor dist
        r, c = max(front_cells, key=lambda rc: self.dist[rc])
        goal_x = c*res + self.info.origin.position.x + res/2
        goal_y = r*res + self.info.origin.position.y + res/2

        # Construir pose
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = goal_x
        pose.pose.position.y = goal_y

        # Orientación = la misma que el yaw actual
        q = quaternion_from_yaw(yaw0)
        pose.pose.orientation = q

        # Enviar goal a Nav2
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
    #              REACCIÓN: Back-Up + Spin
    # ----------------------------------------------------------------------

    def _react(self):
        """
        Cancela el goal y lanza BackUp + Spin para librarse de atascos/obstáculo cercano.
        """
        if not self.back_cli.server_is_ready():
            self.get_logger().warn('BackUp no disponible')
            return

        if self.goal_act:
            self.nav_cli.cancel_all_goals()
            self.goal_act = False

        # Iniciar retroceso
        g = BackUp.Goal()
        g.target = Point(x=-BACK_DIST, y=0.0, z=0.0)
        g.speed  = 0.10
        g.time_allowance.sec = 4
        self.backing = True
        self.back_cli.send_goal_async(g).add_done_callback(self._back_done)

    def _back_done(self, _):
        self.backing = False

        # Tras retroceder, girar
        if not self.spin_cli.server_is_ready():
            return

        s = Spin.Goal()
        s.target_yaw = SPIN_YAW
        s.time_allowance.sec = 6
        self.spinning = True
        self.spin_cli.send_goal_async(s).add_done_callback(self._spin_done)

    def _spin_done(self, _):
        self.spinning = False
        # se queda listo para un nuevo goal en _tick()

# --------------------------------------------------------------------------
#                           MAIN
# --------------------------------------------------------------------------

def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(RandomExplorer())
    rclpy.shutdown()

if __name__ == '__main__':
    main()
