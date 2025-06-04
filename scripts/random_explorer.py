#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
RandomExplorer – versión con ajustes para ir más centrado y reaccionar rápido.
• Objetivo: celda libre y lo más alejada de obstáculos, filtrada por distancia y ángulo.
• Atasco = 8 cm sin progreso durante 5 s → Back-Up 0.2 m + Spin 180 °.
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
STUCK_TIME   = 5.0    # …durante 5 s ⇒ reacción
BACK_DIST    = 0.20   # retroceso 20 cm
SPIN_YAW     = math.pi    # 180 ° (puedes cambiarlo a tu gusto)
COOLDOWN     = 15.0   # mínimo 15 s entre reacciones

# ---------- parámetros de filtración de celdas ----------
MIN_DIST_FROM_OBST = 0.3    # distancia mínima (m) a obstáculos
MAX_ANG            = math.radians(60)  # filtrar celdas en ±60° frente al robot

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
        self.grid = None       # guardará la matriz de celdas del costmap
        self.info = None       # info de resolución y origen del mapa
        self.dist = None       # distance-transform
        self.free = []         # lista de celdas libres (fila, columna)
        self.pose = None       # (x, y, yaw) del robot

        self.goal_act   = False  # indica si hay un goal activo de NavigateToPose
        self.backing    = False  # indica si se está ejecutando una acción de BackUp
        self.spinning   = False  # indica si se está ejecutando una acción de Spin
        self.last_move  = self.get_clock().now()  # última vez que el robot avanzó
        self.last_react = self.get_clock().now()  # última vez que se hizo “reacción” de atasco

        # ── timer principal 1 Hz ──────────────────────────────
        self.create_timer(1.0, self._tick)

    # ----------------------------------------------------------------------
    #                      CALLBACKS
    # ----------------------------------------------------------------------

    def _map_cb(self, msg: OccupancyGrid):
        self.info = msg.info
        # Convertir el array OccupancyGrid en un array de numpy
        self.grid = np.frombuffer(msg.data, dtype=np.int8) \
                     .reshape(msg.info.height, msg.info.width)
        # Celdas libres donde el valor < 10
        self.free = list(zip(*np.where(self.grid < 10)))

        # Calcular distance-transform usando OpenCV,
        # donde obstáculo = 0 y espacio libre = 255
        # Para obstáculos (>= 50), poner 0; para celdas libres, 255
        bin_map = np.where(self.grid >= 50, 0, 255).astype(np.uint8)
        self.dist = cv2.distanceTransform(bin_map, cv2.DIST_L2, 3)

    def _odom_cb(self, msg: Odometry):
        p  = msg.pose.pose.position
        q  = msg.pose.pose.orientation
        # Calcular yaw desde el cuaternión (x, y, z, w)
        yaw = math.atan2(2*(q.w*q.z + q.x*q.y),
                         1 - 2*(q.y*q.y + q.z*q.z))

        # Comprobamos si el robot se ha movido más de STUCK_DIST
        if self.pose and math.hypot(p.x - self.pose[0], p.y - self.pose[1]) > STUCK_DIST:
            self.last_move = self.get_clock().now()

        self.pose = (p.x, p.y, yaw)

    # ----------------------------------------------------------------------
    #                      BUCLE PRINCIPAL
    # ----------------------------------------------------------------------

    def _tick(self):
        # Si estamos en medio de una reacción (back-up o spin), no hacemos nada
        if self.backing or self.spinning:
            return

        now = self.get_clock().now()

        # Chequear si el robot está atascado
        time_since_move  = (now - self.last_move).nanoseconds * 1e-9
        time_since_react = (now - self.last_react).nanoseconds * 1e-9

        stuck = ( self.goal_act and
                  (time_since_move > STUCK_TIME) and
                  (time_since_react > COOLDOWN) )

        if stuck:
            self.last_react = now
            self._react()
            return

        # Si no hay goal activo, enviamos uno nuevo
        if not self.goal_act:
            self._send_goal()

    # ----------------------------------------------------------------------
    #                      ELECCIÓN DE OBJETIVO
    # ----------------------------------------------------------------------

    def _send_goal(self):
        if not (self.nav_cli.server_is_ready() and self.free and
                self.dist is not None and self.pose):
            return

        x0, y0, yaw0 = self.pose
        res = self.info.resolution

        # Filtrar celdas según distancia mínima a obstáculos
        safe_cells = []
        for (r, c) in self.free:
            # dist[r,c] está en píxeles; convertir a metros multiplicando por res
            dist_m = self.dist[r, c] * res
            if dist_m >= MIN_DIST_FROM_OBST:
                safe_cells.append((r, c))

        if not safe_cells:
            self.get_logger().warn('No hay celdas seguras (dist > {:.2f} m).'.format(MIN_DIST_FROM_OBST))
            return

        # Filtrar celdas que estén ±MAX_ANG del frente del robot
        front_cells = []
        for (r, c) in safe_cells:
            wx = c*res + self.info.origin.position.x + res/2
            wy = r*res + self.info.origin.position.y + res/2

            dx = wx - x0
            dy = wy - y0
            angle = math.atan2(dy, dx) - yaw0

            # normalizar ángulo a -pi..pi
            while angle > math.pi:
                angle -= 2*math.pi
            while angle < -math.pi:
                angle += 2*math.pi

            if abs(angle) <= MAX_ANG:
                front_cells.append((r, c))

        if not front_cells:
            self.get_logger().warn('No hay celdas en el frente que cumplan el ángulo ±{:.1f}°.'.format(math.degrees(MAX_ANG)))
            return

        # Elegir la celda con mayor valor de dist
        r, c = max(front_cells, key=lambda rc: self.dist[rc])

        # Convertir (r,c) a coordenadas en el marco "map"
        goal_x = c*res + self.info.origin.position.x + res/2
        goal_y = r*res + self.info.origin.position.y + res/2

        # Construir el mensaje de PoseStamped
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = goal_x
        pose.pose.position.y = goal_y

        # Orientación = la misma que el yaw actual del robot (puedes modificar a conveniencia)
        q = quaternion_from_yaw(yaw0)
        pose.pose.orientation = q

        # Enviamos el goal
        goal = NavigateToPose.Goal()
        goal.pose = pose

        self.goal_act = True
        self.nav_cli.send_goal_async(goal).add_done_callback(self._goal_resp)

    def _goal_resp(self, fut):
        """
        Callback cuando enviamos la meta de NavigateToPose.
        """
        h = fut.result()
        if not h or not h.accepted:
            self.goal_act = False
            return
        h.get_result_async().add_done_callback(self._goal_done)

    def _goal_done(self, fut):
        """
        Callback cuando el goal de NavigateToPose finaliza.
        """
        if fut.result().status != GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info(f'Goal end status {fut.result().status}')
        self.goal_act = False

    # ----------------------------------------------------------------------
    #              REACCIÓN: Back-Up + Spin
    # ----------------------------------------------------------------------

    def _react(self):
        if not self.back_cli.server_is_ready():
            self.get_logger().warn('BackUp no disponible')
            return

        # Cancelar goal activo si lo hubiera
        if self.goal_act:
            self.nav_cli.cancel_all_goals()
            self.goal_act = False

        # Iniciar retroceso
        g = BackUp.Goal()
        g.target = Point(x=-BACK_DIST, y=0.0, z=0.0)  # moverse -0.2 m en X local
        g.speed  = 0.10
        g.time_allowance.sec = 4  # tiempo máximo
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
        # listo para un nuevo goal en el próximo tick

# --------------------------------------------------------------------------
#                           MAIN
# --------------------------------------------------------------------------

def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(RandomExplorer())
    rclpy.shutdown()

if __name__ == '__main__':
    main()
