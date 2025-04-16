#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Float32MultiArray
import random

class UnderwaterIMUSimulator(Node):
    def __init__(self):
        super().__init__('underwater_imu_simulator')

        # Publishers
        self.heading_pub = self.create_publisher(Float32, '/imu/heading', 10)
        self.euler_pub = self.create_publisher(Float32MultiArray, '/imu/euler', 10)

        # State
        self.heading = random.uniform(0, 360)
        self.pitch = 0.0
        self.roll = 0.0
        self.target_pitch = 0.0
        self.target_roll = 0.0

        # ROS timer: update at 20Hz
        self.timer = self.create_timer(0.05, self.update_and_publish)

    def update_and_publish(self):
        # === Heading: light drift + jitter ===
        self.heading += random.uniform(-0.3, 0.3)
        self.heading %= 360

        # === Pitch and Roll: smooth random targets ===
        self._update_orientation()

        # === Publish Heading ===
        heading_msg = Float32()
        heading_msg.data = self.heading
        self.heading_pub.publish(heading_msg)

        # === Publish Euler angles ===
        euler_msg = Float32MultiArray()
        euler_msg.data = [self.roll, self.pitch, 0.0]  # yaw unused
        self.euler_pub.publish(euler_msg)

        # Optional: log occasionally
        if random.random() < 0.1:
            self.get_logger().info(f"Heading: {self.heading:.1f}°, Pitch: {self.pitch:.1f}°, Roll: {self.roll:.1f}°")

    def _update_orientation(self):
        # Randomly adjust target pitch/roll
        self.target_pitch += random.uniform(-1, 1)
        self.target_roll += random.uniform(-1, 1)

        # Clamp to safe visual/demo ranges
        self.target_pitch = max(min(self.target_pitch, 30.0), -30.0)
        self.target_roll = max(min(self.target_roll, 20.0), -20.0)

        # Smooth transition to targets + small noise
        smoothing = 0.05
        noise = lambda scale: random.uniform(-scale, scale)

        self.pitch += (self.target_pitch - self.pitch) * smoothing + noise(0.05)
        self.roll  += (self.target_roll  - self.roll)  * smoothing + noise(0.05)

        # Clamp to ensure valid publishing
        self.pitch = max(min(self.pitch, 90.0), -90.0)
        self.roll  = max(min(self.roll, 90.0), -90.0)

def main(args=None):
    rclpy.init(args=args)
    node = UnderwaterIMUSimulator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
