#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
cone_driver_v6.py – reactivo robusto para ROSbot XL
  • Cono frontal FOV_DEG centrado en CENTER_DEG
  • Mediana de MEDIAN_K mínimos para robustez
  • Tres zonas: despejado (>SLOW), lento (SAFE–SLOW), giro (≤SAFE)
  • /cone_driver/* topics para Foxglove
"""

import rclpy, math, numpy as np
from rclpy.node      import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from std_msgs.msg    import Float32

# -------- PARÁMETROS AFINADOS --------------------------------------
CENTER_DEG   = 106      # dirección frontal real
FOV_DEG      = 140      # ±70°
MEDIAN_K     = 15       # n. rayos para mediana
MIN_VALID    = 0.08     # < 8 cm = ruido
SAFE_DIST    = 0.45
SLOW_DIST    = 1.00
V_MAX        = 0.20
W_GIRO       = 0.7
# -------------------------------------------------------------------

CENTER = math.radians(CENTER_DEG)
HALF   = math.radians(FOV_DEG / 2)

class ConeDriver(Node):
    def __init__(self):
        super().__init__('cone_driver')
        self.pub_cmd = self.create_publisher(Twist, '/cmd_vel', 10)
        self.pub_dbg = {k: self.create_publisher(Float32, f'/cone_driver/{k}', 10)
                        for k in ('min_dist', 'left_min', 'right_min')}
        self.create_subscription(LaserScan, '/scan', self.cb, 10)
        self.tw_L = Twist(); self.tw_L.angular.z =  W_GIRO
        self.tw_R = Twist(); self.tw_R.angular.z = -W_GIRO

    def cb(self, scan: LaserScan):
        rng = np.asarray(scan.ranges, dtype=np.float32)
        ang = np.linspace(scan.angle_min, scan.angle_max, rng.size, dtype=np.float32)

        # rota el eje
        rel = ((ang + math.pi) % (2*math.pi) - math.pi) - CENTER
        rel = (rel + math.pi) % (2*math.pi) - math.pi
        mask = np.abs(rel) <= HALF
        cone = rng[mask]

        # lecturas válidas
        valid = cone[(cone > max(MIN_VALID, scan.range_min)) & (cone < scan.range_max)]
        if valid.size == 0:
            self.publish(V_MAX)
            self.pub_dbg['min_dist'].publish(Float32(data=float('inf')))
            return

        k = min(MEDIAN_K, valid.size)
        dmin = float(np.median(np.partition(valid, k)[:k]))
        self.pub_dbg['min_dist'].publish(Float32(data=dmin))

        # distancias mínimas a izquierda/derecha
        lval = rng[(rel > 0) & mask]
        rval = rng[(rel < 0) & mask]
        lmin = float(np.min(lval[lval > MIN_VALID])) if np.any(lval > MIN_VALID) else 0.0
        rmin = float(np.min(rval[rval > MIN_VALID])) if np.any(rval > MIN_VALID) else 0.0
        self.pub_dbg['left_min' ].publish(Float32(data=lmin))
        self.pub_dbg['right_min'].publish(Float32(data=rmin))

        # decisión
        if dmin <= SAFE_DIST:
            self.pub_cmd.publish(self.tw_L if lmin > rmin else self.tw_R)
        elif dmin < SLOW_DIST:
            vel = V_MAX * (dmin - SAFE_DIST)/(SLOW_DIST - SAFE_DIST)
            self.publish(max(0.05, vel))
        else:
            self.publish(V_MAX)

    def publish(self, v=0.0, w=0.0):
        tw = Twist(); tw.linear.x = v; tw.angular.z = w
        self.pub_cmd.publish(tw)

def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(ConeDriver())
    rclpy.shutdown()
if __name__ == '__main__':
    main()
