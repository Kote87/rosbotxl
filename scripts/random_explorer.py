#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
cone_driver_v2.py
Control reactivo sencillísimo:
  • Usa /scan (LaserScan)
  • Solo considera un cono frontal de FOV_DEG (por defecto 120 °)
  • Avanza recto si ningún rayo válido del cono < SAFE_DIST_M
  • Si hay obstáculo < SAFE_DIST_M ⇒ gira en el sitio hacia el lado con más espacio
  • Publica /cone_driver/min_dist (Float32) para depuración
"""

import rclpy, math, numpy as np
from rclpy.node         import Node
from sensor_msgs.msg    import LaserScan
from geometry_msgs.msg  import Twist
from std_msgs.msg       import Float32

# ---------- AJUSTA SOLO ESTAS 4 VARIABLES ---------------------------
CENTER_DEG    = 90      # ángulo (deg) que apunta REALMENTE al frente del robot
FOV_DEG       = 120     # ancho total del haz (p. e. 120 ° → ±60 °)
SAFE_DIST_M   = 0.20    # si algo < SAFE_DIST → girar
FWD_SPEED_MPS = 0.18    # m/s
ANG_SPEED_RPS = 0.6     # rad/s
# --------------------------------------------------------------------

CENTER_RAD = math.radians(CENTER_DEG)
HALF_FOV   = math.radians(FOV_DEG/2)

class ConeDriver(Node):
    def __init__(self):
        super().__init__('cone_driver')
        # Publicadores
        self.cmd_pub   = self.create_publisher(Twist, '/cmd_vel', 10)
        self.debug_pub = self.create_publisher(Float32, '/cone_driver/min_dist', 10)
        # Subscripción al LIDAR
        self.create_subscription(LaserScan, '/scan', self.scan_cb, 10)

        # Pre‑definimos Twists
        self.tw_fwd            = Twist(); self.tw_fwd.linear.x   = FWD_SPEED_MPS
        self.tw_left, self.tw_right = Twist(), Twist()
        self.tw_left.angular.z  =  ANG_SPEED_RPS
        self.tw_right.angular.z = -ANG_SPEED_RPS

    def scan_cb(self, scan: LaserScan):
        rng = np.asarray(scan.ranges, dtype=np.float32)
        ang = np.linspace(scan.angle_min, scan.angle_max, rng.size, dtype=np.float32)

        # paso 1: convertir ángulos a [-π, π)
        ang = (ang + math.pi) % (2*math.pi) - math.pi
        # paso 2: desplazar para que CENTER_RAD quede como “0”
        rel = (ang - CENTER_RAD + math.pi) % (2*math.pi) - math.pi

        # Cono frontal
        mask = np.abs(rel) <= HALF_FOV
        cone_ranges = rng[mask]

        # Filtra lecturas fuera de rango (inf, 0, nan…)
        valid = np.logical_and(
                    cone_ranges > scan.range_min,
                    cone_ranges < scan.range_max)
        valid_ranges = cone_ranges[valid]

        # Si no hay lecturas válidas, asumimos libre
        if valid_ranges.size == 0:
            self.cmd_pub.publish(self.tw_fwd)
            self.debug_pub.publish(Float32(data=float('inf')))
            return

        d_min = float(np.min(valid_ranges))
        self.debug_pub.publish(Float32(data=d_min))

        if d_min >= SAFE_DIST_M:
            # No hay obstáculo en el cono
            self.cmd_pub.publish(self.tw_fwd)
            return

        # Hay obstáculo: decide dirección de giro
        left_mask  = np.logical_and(rel >  0, mask)
        right_mask = np.logical_and(rel <  0, mask)

        left_vals  = rng[left_mask]
        right_vals = rng[right_mask]

        # Filtra cada lado
        lv = left_vals[(left_vals > scan.range_min) & (left_vals < scan.range_max)]
        rv = right_vals[(right_vals > scan.range_min) & (right_vals < scan.range_max)]

        left_mean  = np.mean(lv) if lv.size else 0.0
        right_mean = np.mean(rv) if rv.size else 0.0

        if left_mean >= right_mean:
            self.cmd_pub.publish(self.tw_left)
        else:
            self.cmd_pub.publish(self.tw_right)

def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(ConeDriver())
    rclpy.shutdown()

if __name__ == '__main__':
    main()
