#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan, Range
import matplotlib.pyplot as plt
import numpy as np
from collections import deque
import threading
import time

class SensorLivePlot(Node):
    def __init__(self):
        super().__init__('sensor_live_plot')

        # Subscribers
        self.create_subscription(LaserScan, '/scan', self.lidar_callback, qos_profile_sensor_data)
        self.create_subscription(Range, '/ultrasonic/front', self.ultrasonic_callback, 10)

        # Buffers
        self.lidar_theta = None
        self.lidar_ranges = None
        self.new_lidar_data = False

        self.start_time = time.time()
        self.ultra_times = deque(maxlen=100)
        self.ultra_ranges = deque(maxlen=100)

    def lidar_callback(self, msg: LaserScan):
        # We keep theta in Radians for the Polar plot
        if self.lidar_theta is None or len(self.lidar_theta) != len(msg.ranges):
            self.lidar_theta = np.linspace(msg.angle_min, msg.angle_max, len(msg.ranges))
            
        # Clean data: Replace inf with max range
        ranges = np.array(msg.ranges)
        ranges[np.isinf(ranges)] = msg.range_max
        self.lidar_ranges = ranges
        self.new_lidar_data = True

    def ultrasonic_callback(self, msg: Range):
        self.ultra_times.append(time.time() - self.start_time)
        self.ultra_ranges.append(msg.range)

def main():
    rclpy.init()
    node = SensorLivePlot()

    # Spin in background
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    # Setup Figures
    fig = plt.figure(figsize=(10, 6))
    
    # 1. Polar Plot for LiDAR
    ax_lidar = fig.add_subplot(121, projection='polar')
    ax_lidar.set_title("LiDAR Scan (Radar View)")
    # Note: Many LiDARs have 0 degrees at the front. 
    # Use set_theta_offset(np.pi/2.0) if you want 0 to be "North"
    lidar_scatter = ax_lidar.scatter([], [], s=2, c='b') 
    ax_lidar.set_ylim(0, 6.0) # Set max distance to 6 meters

    # 2. Linear Plot for Ultrasonic
    ax_ultra = fig.add_subplot(122)
    ax_ultra.set_title("Ultrasonic History")
    ax_ultra.set_ylabel("Distance (m)")
    ultra_line, = ax_ultra.plot([], [], 'r-')
    ax_ultra.set_ylim(0, 4.0)

    plt.tight_layout()

    try:
        while rclpy.ok():
            if node.new_lidar_data:
                # To update a scatter plot efficiently:
                # We combine theta and r into (N, 2) offsets
                offsets = np.column_stack((node.lidar_theta, node.lidar_ranges))
                lidar_scatter.set_offsets(offsets)
                node.new_lidar_data = False

            if len(node.ultra_times) > 0:
                ultra_line.set_data(list(node.ultra_times), list(node.ultra_ranges))
                ax_ultra.set_xlim(max(0, node.ultra_times[-1] - 10), node.ultra_times[-1] + 1)

            plt.pause(0.05) # Refresh at ~20fps

    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()