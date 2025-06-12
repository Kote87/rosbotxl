#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
explorer_node.py – Calibración y reactive driver para ROSbot XL con RPLIDAR S3
  • Calibra el centro del LiDAR al iniciar: rota en sitio y calcula offset.
  • Luego usa controlador reactivo “cono” adaptado con center calibrado.
"""

import rclpy
import math
import numpy as np
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

# Parámetros ajustables
FOV_DEG      = 140      # campo de visión frontal ±70°
MEDIAN_K     = 15       # mediana de los k mínimos
MIN_VALID    = 0.08     # < 8 cm se considera ruido
SAFE_DIST    = 0.45     # distancia límite de giro
SLOW_DIST    = 1.00     # distancia límite de desaceleración
V_MAX        = 0.20     # velocidad máxima (m/s)
W_GIRO       = 0.7      # velocidad angular de giro (rad/s)

# Parámetros de calibración
CALIB_SAMPLES = 100     # muestras de escaneo para calibrar
ROT_SPEED     = W_GIRO  # velocidad de rotación en calibración
MSG_FREQ      = 10      # frecuencia de publicación (Hz)

class Explorer(Node):
    def __init__(self):
        super().__init__('explorer')
        self.pub_cmd = self.create_publisher(Twist, '/cmd_vel', 10)

        # Estado de calibración
        self.calib_readings = []
        self.calibrated = False
        self.center = None  # offset del centro en radianes

        # Suscripción y timer para calibración
        self.sub_calib = self.create_subscription(
            LaserScan, '/scan', self.cb_calib, 10)
        self.timer = self.create_timer(1.0/MSG_FREQ, self.pub_rotate)

    def pub_rotate(self):
        # Mientras no esté calibrado, rota en sitio
        if not self.calibrated:
            tw = Twist()
            tw.angular.z = ROT_SPEED
            self.pub_cmd.publish(tw)

    def cb_calib(self, scan: LaserScan):
        # Extrae índice y ángulo de la lectura mínima válida
        rng = np.array(scan.ranges, dtype=np.float32)
        valid = np.where((rng > max(MIN_VALID, scan.range_min)) &
                          (rng < scan.range_max))[0]
        if valid.size == 0:
            return
        imin = valid[np.argmin(rng[valid])]
        angle = scan.angle_min + imin * scan.angle_increment
        self.calib_readings.append(angle)

        # Cuando alcanza el número de muestras, calcula el centro
        if len(self.calib_readings) >= CALIB_SAMPLES:
            angles = np.array(self.calib_readings)
            # Media circular
            x = np.mean(np.cos(angles))
            y = np.mean(np.sin(angles))
            self.center = math.atan2(y, x)
            self.calibrated = True
            self.get_logger().info(
                f'Calibración completa: center = {math.degrees(self.center):.2f}°')

            # Limpieza y paso al driver principal
            self.destroy_subscription(self.sub_calib)
            self.timer.cancel()
            self.create_subscription(LaserScan, '/scan',
                                     self.cb_main, 10)

    def cb_main(self, scan: LaserScan):
        # Driver reactivo “cono” usando self.center calibrado
        rng = np.array(scan.ranges, dtype=np.float32)
        ang = np.linspace(scan.angle_min, scan.angle_max,
                          rng.size, dtype=np.float32)

        # Ajusta los ángulos relativos al centro
        rel = ((ang - self.center + math.pi) % (2*math.pi)) - math.pi
        mask = np.abs(rel) <= math.radians(FOV_DEG/2)
        cone = rng[mask]

        # Lecturas válidas
        valid = cone[(cone > max(MIN_VALID, scan.range_min)) &
                     (cone < scan.range_max)]
        if valid.size == 0:
            self.publish(V_MAX, 0.0)
            return

        # Distancia mínima (mediana de k mínimos)
        k = min(MEDIAN_K, valid.size)
        dmin = float(np.median(np.partition(valid, k)[:k]))

        # Distancias mínimas a izquierda y derecha
        lmask = (rel > 0) & mask
        rmask = (rel < 0) & mask
        lval = rng[lmask]
        rval = rng[rmask]
        lmin = float(np.min(lval[lval > MIN_VALID])) if np.any(lval > MIN_VALID) else float('inf')
        rmin = float(np.min(rval[rval > MIN_VALID])) if np.any(rval > MIN_VALID) else float('inf')

        # Lógica de movimiento
        if dmin <= SAFE_DIST:
            # Giro sobre el punto más despejado
            if lmin > rmin:
                self.publish(0.0, W_GIRO)
            else:
                self.publish(0.0, -W_GIRO)
        elif dmin < SLOW_DIST:
            # Rampa de desaceleración suave
            vel = V_MAX * (dmin - SAFE_DIST) / (SLOW_DIST - SAFE_DIST)
            self.publish(max(0.0, vel), 0.0)
        else:
            # Avance a velocidad máxima
            self.publish(V_MAX, 0.0)

    def publish(self, lin_x, ang_z):
        tw = Twist()
        tw.linear.x = lin_x
        tw.angular.z = ang_z
        self.pub_cmd.publish(tw)


def main(args=None):
    rclpy.init(args=args)
    node = Explorer()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
