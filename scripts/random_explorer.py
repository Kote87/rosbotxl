#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
cone_driver_v7.py  – reactivo mejorado
  • Frena sólo entre 0.60 m y 0.35 m
  • Si d ≤ SAFE_DIST se PARA y gira in‑place
  • Tras giro envía impulso lateral 0.06 m/s 0.3 s
"""

import rclpy, math, numpy as np, time
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32

# ---------- parámetros rápidos ---------------------------------
CENTER_DEG = 106
FOV_DEG = 140  # ±70°
SAFE_DIST = 0.35
SLOW_DIST = 0.60
V_MAX = 0.22
W_TURN = 1.0  # rad/s
PUSH_TIME = 0.30
PUSH_VEL = 0.06
MEDIAN_K = 15
MIN_VALID = 0.08
# ---------------------------------------------------------------

C = math.radians(CENTER_DEG)
H = math.radians(FOV_DEG / 2)


class ConeDriver(Node):
    def __init__(self):
        super().__init__("cone_driver")
        self.pub_cmd = self.create_publisher(Twist, "/cmd_vel", 10)
        self.pub_dbg = {
            k: self.create_publisher(Float32, f"/cone_driver/{k}", 10)
            for k in ("min_dist",)
        }
        self.create_subscription(LaserScan, "/scan", self.cb, 10)

        # mensajes precocinados
        self.tw_L = Twist()
        self.tw_L.angular.z = W_TURN
        self.tw_R = Twist()
        self.tw_R.angular.z = -W_TURN

        self.push_until = 0.0  # para impulso post‑giro

    # -----------------------------------------------------------
    def cb(self, scan: LaserScan):
        rng = np.asarray(scan.ranges, dtype=np.float32)
        ang = np.linspace(scan.angle_min, scan.angle_max, rng.size, dtype=np.float32)

        rel = ((ang + math.pi) % (2 * math.pi) - math.pi) - C
        rel = (rel + math.pi) % (2 * math.pi) - math.pi
        mask = np.abs(rel) <= H
        cone = rng[mask]

        valid = cone[(cone > max(MIN_VALID, scan.range_min)) & (cone < scan.range_max)]
        if valid.size == 0:
            self.publish(V_MAX)
            return

        dmin = float(np.median(np.partition(valid, min(MEDIAN_K, valid.size))))
        self.pub_dbg["min_dist"].publish(Float32(data=dmin))

        # ↓↓↓ decisión principal ↓↓↓
        if dmin <= SAFE_DIST:  # peligro
            self.push_until = time.time() + PUSH_TIME
            self.pub_cmd.publish(self.tw_L)  # elige giro fijo antihorario
        elif time.time() < self.push_until:  # impulso para despegar
            self.publish(PUSH_VEL, 0.0)
        elif dmin < SLOW_DIST:  # tramo lento
            vel = V_MAX * (dmin - SAFE_DIST) / (SLOW_DIST - SAFE_DIST)
            self.publish(max(0.05, vel))
        else:  # despejado
            self.publish(V_MAX)

    def publish(self, v=0.0, w=0.0):
        tw = Twist()
        tw.linear.x = v
        tw.angular.z = w
        self.pub_cmd.publish(tw)


def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(ConeDriver())
    rclpy.shutdown()


if __name__ == "__main__":
    main()
