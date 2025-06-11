#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
cone_driver_v3.py — control reactivo para ROSbot XL
 • Analiza LaserScan en un cono frontal (FOV_DEG) centrado en CENTER_DEG
 • Velocidad lineal proporcional a la distancia libre
 • Gira cuando un obstáculo entra a SAFE_DIST
 • Publica /cone_driver/{min_dist,left_min,right_min} para depurar
"""

import rclpy, math, numpy as np
from rclpy.node      import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from std_msgs.msg    import Float32

# ========== PARÁMETROS QUE SUELEN NECESITAR AJUSTE ===================
CENTER_DEG      = 90        # 0 si tu scan apunta al eje X; 90 si apunta +Y
FOV_DEG         = 120       # ancho total del haz (120 ° → ±60 °)
SAFE_DIST_M     = 0.25      # a esta distancia se detiene y gira
SLOW_DIST_M     = 0.60      # empieza a reducir velocidad si obstáculo < SLOW_DIST
FWD_MAX_MPS     = 0.25      # velocidad máxima de avance
ANG_SPEED_RPS   = 0.6       # velocidad angular mientras gira
# =====================================================================

CENTER_RAD = math.radians(CENTER_DEG)
HALF_FOV   = math.radians(FOV_DEG / 2)

class ConeDriver(Node):
    def __init__(self):
        super().__init__('cone_driver')
        # Publishers
        self.cmd_pub   = self.create_publisher(Twist, '/cmd_vel', 10)
        self.pub_min   = self.create_publisher(Float32, '/cone_driver/min_dist',   10)
        self.pub_lmin  = self.create_publisher(Float32, '/cone_driver/left_min',   10)
        self.pub_rmin  = self.create_publisher(Float32, '/cone_driver/right_min',  10)

        # Subscriber
        self.create_subscription(LaserScan, '/scan', self.scan_cb, 10)

        # Twist pre‑creados (solo rellenamos campos necesarios)
        self.tw_rotate_l = Twist(); self.tw_rotate_l.angular.z  =  ANG_SPEED_RPS
        self.tw_rotate_r = Twist(); self.tw_rotate_r.angular.z  = -ANG_SPEED_RPS

    # ---------- CALLBACK DE LÁSER ------------------------------------
    def scan_cb(self, scan: LaserScan):
        ranges = np.asarray(scan.ranges, dtype=np.float32)
        angles = np.linspace(scan.angle_min, scan.angle_max, ranges.size, dtype=np.float32)

        # Ajusta ángulos a [-π,π) y luego los rota para que CENTER_RAD sea “0”
        rel = ((angles + math.pi) % (2*math.pi) - math.pi) - CENTER_RAD
        rel = (rel + math.pi) % (2*math.pi) - math.pi      # renormaliza

        mask_cone = np.abs(rel) <= HALF_FOV                # rayos dentro del cono
        cone_ranges = ranges[mask_cone]

        # Filtra lecturas válidas
        valid = np.logical_and(cone_ranges > scan.range_min,
                               cone_ranges < scan.range_max)
        valid_ranges = cone_ranges[valid]

        # Si no hay lecturas válidas ⇒ asumimos todo despejado
        if valid_ranges.size == 0:
            self.publish_cmd(linear=FWD_MAX_MPS)
            self.pub_min.publish(Float32(data=float('inf')))
            self.pub_lmin.publish(Float32(data=float('inf')))
            self.pub_rmin.publish(Float32(data=float('inf')))
            return

        d_min = float(np.min(valid_ranges))
        self.pub_min.publish(Float32(data=d_min))

        # --------------------------------------------------------------
        # DISTANCIAS LATERALES (para elegir sentido de giro)
        left_mask  = np.logical_and(rel > 0, mask_cone)
        right_mask = np.logical_and(rel < 0, mask_cone)

        l_vals = ranges[left_mask]
        r_vals = ranges[right_mask]

        l_vals = l_vals[(l_vals > scan.range_min) & (l_vals < scan.range_max)]
        r_vals = r_vals[(r_vals > scan.range_min) & (r_vals < scan.range_max)]

        l_min = float(np.min(l_vals))  if l_vals.size else 0.0
        r_min = float(np.min(r_vals))  if r_vals.size else 0.0
        self.pub_lmin.publish(Float32(data=l_min))
        self.pub_rmin.publish(Float32(data=r_min))
        # --------------------------------------------------------------

        # 1. Obstáculo MUY cerca  → girar
        if d_min <= SAFE_DIST_M:
            cmd = self.tw_rotate_l if l_min > r_min else self.tw_rotate_r
            self.cmd_pub.publish(cmd)
            return

        # 2. Obstáculo a media distancia → avanza más lento
        if d_min < SLOW_DIST_M:
            # velocidad lineal proporcional (entre 0 y FWD_MAX)
            v = FWD_MAX_MPS * (d_min - SAFE_DIST_M) / (SLOW_DIST_M - SAFE_DIST_M)
            v = max(0.05, v)                           # al menos 5 cm/s
            self.publish_cmd(linear=v)
            return

        # 3. Camino despejado → velocidad máxima
        self.publish_cmd(linear=FWD_MAX_MPS)

    # ---------- UTIL -------------------------------------------------
    def publish_cmd(self, *, linear=0.0, angular=0.0):
        tw = Twist()
        tw.linear.x  = linear
        tw.angular.z = angular
        self.cmd_pub.publish(tw)

def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(ConeDriver())
    rclpy.shutdown()

if __name__ == '__main__':
    main()
