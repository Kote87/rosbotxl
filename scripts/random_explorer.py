#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
simple_cone_driver.py
Control reactivo:
 - Usa LaserScan (/scan)
 - Cono frontal ±60 ° (120 °)
 - Avanza a 0.18 m/s si no hay obstáculo < 0.20 m
 - Si hay obstáculo <0.20 m → gira (0.6 rad/s) hacia el lado con más espacio
"""

import rclpy, math, numpy as np
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

FOV_DEG       = 120               # campo frontal total
SAFE_DIST_M   = 0.20              # distancia mínima (m) para avanzar
FWD_SPEED_MPS = 0.18              # velocidad lineal cuando avanza
ANG_SPEED_RPS = 0.6               # velocidad de giro cuando esquiva

class ConeDriver(Node):
    def __init__(self):
        super().__init__('cone_driver')
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(LaserScan, '/scan', self.scan_cb, 10)

        # Pre‑creamos mensajes Twist para ahorrar tiempo
        self.twist_fwd   = Twist(); self.twist_fwd.linear.x = FWD_SPEED_MPS
        self.twist_left  = Twist(); self.twist_left.angular.z  =  ANG_SPEED_RPS
        self.twist_right = Twist(); self.twist_right.angular.z = -ANG_SPEED_RPS
        self.twist_stop  = Twist()                              # todo cero

    def scan_cb(self, scan: LaserScan):
        """Procesa cada LaserScan y decide cmd_vel."""
        # Array de distancias
        ranges = np.array(scan.ranges, dtype=np.float32)
        # Array de ángulos (rad)
        angles = np.linspace(scan.angle_min,
                             scan.angle_max,
                             len(ranges), dtype=np.float32)

        # Cono frontal: |ángulo| ≤ FOV/2
        mask = np.abs(angles) <= math.radians(FOV_DEG/2)
        front_dist = ranges[mask]

        # ¿hay algo más cerca que SAFE_DIST_M?
        if np.any(front_dist < SAFE_DIST_M):
            # Obstáculo: elegimos lado con más espacio medio
            left_space  = np.mean(ranges[(angles>0)  & mask])
            right_space = np.mean(ranges[(angles<0)  & mask])
            if np.isinf(left_space):  left_space  = 10.0
            if np.isinf(right_space): right_space = 10.0

            # Gira hacia el lado con MAYOR distancia
            cmd = self.twist_left if left_space > right_space else self.twist_right
            self.cmd_pub.publish(cmd)
        else:
            # Camino libre → avanzar recto
            self.cmd_pub.publish(self.twist_fwd)

def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(ConeDriver())
    rclpy.shutdown()

if __name__ == '__main__':
    main()
