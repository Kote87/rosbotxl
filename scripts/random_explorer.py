#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Explorador que ignora lo que hay detrás (±90°) y solo avanza.
 - Suscribe costmap global / global_costmap/costmap
 - Suscribe odometría /odometry/filtered
 - Filtra obstáculos "detrás" del robot
 - Escoge meta en 180° frontales (±90°) y la envía a Nav2
 - No hay maniobras de back-up ni spin
 - Si no encuentra celdas libres delante, no avanza
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
import math
import numpy as np
import cv2

from geometry_msgs.msg import PoseStamped, Quaternion
from nav_msgs.msg      import OccupancyGrid, Odometry
from nav2_msgs.action  import NavigateToPose
from action_msgs.msg   import GoalStatus

# ----- Parámetros de exploración -----
MIN_DIST_FROM_OBST = 0.3           # Mínimo 30 cm de distancia al obstáculo
MAX_ANG            = math.radians(90)  # ±90° => 180° en total
MAP_FREE_THRESHOLD = 10            # celdas < 10 se consideran libres

def quaternion_from_yaw(yaw: float) -> Quaternion:
    q = Quaternion()
    q.z = math.sin(yaw/2)
    q.w = math.cos(yaw/2)
    return q

class ForwardExplorer(Node):
    def __init__(self):
        super().__init__('forward_explorer')
        # Action client para NavigateToPose (Nav2)
        self.nav_cli = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # Subscripciones
        self.create_subscription(OccupancyGrid, '/global_costmap/costmap',
                                 self._map_cb, 10)
        self.create_subscription(Odometry, '/odometry/filtered',
                                 self._odom_cb, 20)

        # Buffers
        self.grid   = None
        self.info   = None
        self.dist   = None
        self.pose   = None   # (x, y, yaw)
        self.free   = []     # celdas consideradas libres
        self.goal_active = False

        # Timer principal (1 Hz)
        self.create_timer(1.0, self._tick)

    # ------------------------------------------------------------------
    #                  CALLBACKS
    # ------------------------------------------------------------------
    def _map_cb(self, msg: OccupancyGrid):
        """
        Recibimos el costmap global.
        1) Convertir a array numpy.
        2) Filtrar obstáculos traseros (±90°).
        3) Actualizar 'free' y 'dist' con distanceTransform.
        """
        self.info = msg.info
        w = self.info.width
        h = self.info.height

        # Convertimos a numpy
        grid_np = np.frombuffer(msg.data, dtype=np.int8).reshape(h, w)

        # Si tenemos la pose, ignorar obst. detrás
        if self.pose:
            x_r, y_r, yaw_r = self.pose
            for r in range(h):
                for c in range(w):
                    if grid_np[r, c] >= 50:
                        # coordenadas en el mundo
                        wx = c*self.info.resolution + self.info.origin.position.x + self.info.resolution/2
                        wy = r*self.info.resolution + self.info.origin.position.y + self.info.resolution/2

                        dx = wx - x_r
                        dy = wy - y_r

                        angle = math.atan2(dy, dx) - yaw_r
                        # normalizar
                        while angle > math.pi:
                            angle -= 2*math.pi
                        while angle < -math.pi:
                            angle += 2*math.pi

                        # Si está fuera de ±90° => está "detrás" => anular
                        if abs(angle) > math.pi/2:
                            grid_np[r, c] = 0  # marcar como libre

        # Guardamos
        self.grid = grid_np

        # Celdas libres
        self.free = list(zip(*np.where(self.grid < MAP_FREE_THRESHOLD)))

        # DistTransform
        bin_map = np.where(self.grid >= 50, 0, 255).astype(np.uint8)
        self.dist = cv2.distanceTransform(bin_map, cv2.DIST_L2, 3)

    def _odom_cb(self, msg: Odometry):
        """
        Guardar la pose (x, y, yaw)
        """
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation

        yaw = math.atan2(2*(q.w*q.z + q.x*q.y),
                         1 - 2*(q.y*q.y + q.z*q.z))
        self.pose = (p.x, p.y, yaw)

    # ------------------------------------------------------------------
    #               BUCLE PRINCIPAL
    # ------------------------------------------------------------------
    def _tick(self):
        """
        1) Si hay goal activo, no hacer nada.
        2) Sino, enviamos un nuevo goal si es que hay celdas frontales.
        """
        if self.goal_active:
            return

        self._send_goal()

    # ------------------------------------------------------------------
    #               ELECCIÓN DE OBJETIVO
    # ------------------------------------------------------------------
    def _send_goal(self):
        """
        Filtra celdas seguras (dist >= MIN_DIST_FROM_OBST) y en ±90°.
        Elige la de mayor distanceTransform y envía un NavigateToPose.
        """
        if not (self.nav_cli.server_is_ready() and
                self.grid is not None and
                self.dist is not None and
                self.pose is not None and
                self.free):
            return

        x_r, y_r, yaw_r = self.pose
        res = self.info.resolution

        # 1) Filtrar celdas según MIN_DIST_FROM_OBST
        safe_cells = []
        for (r, c) in self.free:
            d_m = self.dist[r, c] * res
            if d_m >= MIN_DIST_FROM_OBST:
                safe_cells.append((r, c))

        if not safe_cells:
            self.get_logger().warn('No hay celdas seguras delante')
            return

        # 2) Filtrar en ±90° con respecto al frente
        front_cells = []
        for (r, c) in safe_cells:
            wx = c*res + self.info.origin.position.x + res/2
            wy = r*res + self.info.origin.position.y + res/2

            dx = wx - x_r
            dy = wy - y_r
            angle = math.atan2(dy, dx) - yaw_r
            while angle > math.pi:
                angle -= 2*math.pi
            while angle < -math.pi:
                angle += 2*math.pi

            if abs(angle) <= MAX_ANG:
                front_cells.append((r, c))

        if not front_cells:
            self.get_logger().warn('No hay celdas en ±90° delante')
            return

        # 3) Elegir la celda con mayor dist
        r, c = max(front_cells, key=lambda rc: self.dist[rc])

        goal_x = c*res + self.info.origin.position.x + res/2
        goal_y = r*res + self.info.origin.position.y + res/2

        # Construir el PoseStamped
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp    = self.get_clock().now().to_msg()
        pose.pose.position.x = goal_x
        pose.pose.position.y = goal_y

        # Orientación la dejamos igual al yaw actual
        q = quaternion_from_yaw(yaw_r)
        pose.pose.orientation = q

        # Enviar a Nav2
        goal = NavigateToPose.Goal()
        goal.pose = pose

        self.goal_active = True
        send = self.nav_cli.send_goal_async(goal)
        send.add_done_callback(self._goal_resp)

    def _goal_resp(self, fut):
        """
        Respuesta a la petición de goal:
         - Si aceptado => esperamos resultado
         - Si no => goal_active=False => volverá a intentarlo
        """
        h = fut.result()
        if not h or not h.accepted:
            self.goal_active = False
            return

        h.get_result_async().add_done_callback(self._goal_done)

    def _goal_done(self, fut):
        """
        Al terminar la navegación => liberamos el flag y a la próxima
        iteración en _tick() se intentará un nuevo goal.
        """
        if fut.result().status != GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info(f'Goal end status {fut.result().status}')
        self.goal_active = False

# ------------------------------------------------------------------
def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(ForwardExplorer())
    rclpy.shutdown()

if __name__ == '__main__':
    main()
