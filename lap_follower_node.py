#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import Point

import numpy as np
import math
import time
from scipy.ndimage import gaussian_filter1d

class LapFollowerNode(Node):
    def __init__(self):
        super().__init__('lap_follower_node')

        self.drive_pub = self.create_publisher(AckermannDriveStamped, '/drive', 10)
        self.create_subscription(LaserScan, '/scan', self.lidar_callback, 10)
        self.create_subscription(Odometry, '/ego_racecar/odom', self.odom_callback, 10)

        # Línea de meta (ajústala con tus coordenadas)
        self.start_line = Point(x=0.0, y=0.0, z=0.0)
        self.cross_radius = 1.0
        self.left_start_zone = True

        # Control de vueltas
        self.lap_count = 0
        self.lap_start_time = time.time()

        # Parámetros de control
        self.max_speed = 12.0
        self.min_speed = 0.5
        self.min_gap_threshold = 1.5
        self.angle_scale = 1.5  # controla sensibilidad a giros

        self.get_logger().info("🔧 LapFollowerNode optimizado iniciado.")

    def lidar_callback(self, scan: LaserScan):
        ranges = np.array(scan.ranges)
        ranges[np.isnan(ranges)] = 0.0
        ranges[np.isinf(ranges)] = scan.range_max

        # Aplicar suavizado Gaussiano
        smooth_ranges = gaussian_filter1d(ranges, sigma=2)

        # Determinar región frontal amplia (aprox ±90°)
        total_indices = len(smooth_ranges)
        start_idx = total_indices // 4
        end_idx = 3 * total_indices // 4
        front_ranges = smooth_ranges[start_idx:end_idx]

        # Identificar los índices del gap más largo por encima del umbral
        valid = front_ranges > self.min_gap_threshold
        max_gap = self.find_largest_gap(valid)

        if max_gap is None:
            steering_angle = 0.0
            speed = self.min_speed
        else:
            gap_start, gap_end = max_gap
            gap_center = (gap_start + gap_end) // 2
            global_index = start_idx + gap_center

            angle = scan.angle_min + global_index * scan.angle_increment
            steering_angle = angle / self.angle_scale

            # Velocidad adaptativa: menos velocidad si el giro es pronunciado
            angle_mag = abs(steering_angle)
            speed = max(self.min_speed, self.max_speed * (1.0 - angle_mag))

        # Publicar mensaje
        msg = AckermannDriveStamped()
        msg.drive.steering_angle = float(steering_angle)
        msg.drive.speed = float(speed)
        self.drive_pub.publish(msg)

    def find_largest_gap(self, valid_mask):
        """
        Devuelve (inicio, fin) del mayor segmento continuo donde valid_mask == True
        """
        max_len = 0
        max_range = None
        current_start = None

        for i, valid in enumerate(valid_mask):
            if valid:
                if current_start is None:
                    current_start = i
            else:
                if current_start is not None:
                    current_len = i - current_start
                    if current_len > max_len:
                        max_len = current_len
                        max_range = (current_start, i - 1)
                    current_start = None

        # Por si termina en True
        if current_start is not None:
            current_len = len(valid_mask) - current_start
            if current_len > max_len:
                max_range = (current_start, len(valid_mask) - 1)

        return max_range

    def odom_callback(self, msg: Odometry):
        position = msg.pose.pose.position
        dist = self.euclidean_distance(position, self.start_line)

        if dist > self.cross_radius:
            self.left_start_zone = True

        if dist < self.cross_radius and self.left_start_zone:
            self.left_start_zone = False
            self.lap_count += 1
            now = time.time()
            lap_time = now - self.lap_start_time
            self.lap_start_time = now

            self.get_logger().info(f"✅ Vuelta {self.lap_count} completada en {lap_time:.2f} segundos")

    @staticmethod
    def euclidean_distance(p1, p2):
        return math.sqrt((p1.x - p2.x)**2 + (p1.y - p2.y)**2)

def main(args=None):
    rclpy.init(args=args)
    node = LapFollowerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
