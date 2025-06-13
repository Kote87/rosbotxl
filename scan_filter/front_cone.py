#!/usr/bin/env python3
import rclpy, math, numpy as np
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

class FrontCone(Node):
    def __init__(self):
        super().__init__('scan_front_filter')
        self.declare_parameter('center_deg', 106.0)
        self.declare_parameter('fov_deg', 120.0)
        self.pub = self.create_publisher(LaserScan, '/scan_front', 10)
        self.create_subscription(LaserScan, '/scan', self.cb, 10)

    def cb(self, msg):
        c = math.radians(self.get_parameter('center_deg').value)
        h = math.radians(self.get_parameter('fov_deg').value / 2)
        n = len(msg.ranges)
        a = np.linspace(msg.angle_min, msg.angle_max, n, dtype=np.float32)
        rel = ((a + math.pi) % (2 * math.pi) - math.pi) - c
        rel = (rel + math.pi) % (2 * math.pi) - math.pi
        mask = np.abs(rel) <= h

        out = LaserScan()
        out.header = msg.header
        out.angle_min = -h
        out.angle_max = +h
        out.angle_increment = msg.angle_increment
        out.time_increment = msg.time_increment
        out.scan_time = msg.scan_time
        out.range_min = msg.range_min
        out.range_max = msg.range_max
        out.ranges = np.asarray(msg.ranges)[mask].tolist()
        self.pub.publish(out)

def main():
    rclpy.init(); rclpy.spin(FrontCone()); rclpy.shutdown()

if __name__ == '__main__':
    main()
