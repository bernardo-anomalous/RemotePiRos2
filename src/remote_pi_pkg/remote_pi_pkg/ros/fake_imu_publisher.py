#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Float32MultiArray
import math
import time

class FakeIMUPublisher(Node):
    def __init__(self):
        super().__init__('fake_imu_publisher')
        self.euler_pub = self.create_publisher(Float32MultiArray, 'imu/euler', 10)
        self.heading_pub = self.create_publisher(Float32, 'imu/heading', 10)
        self.timer = self.create_timer(0.05, self.publish_fake_data)  # 20Hz
        self.start_time = time.time()

    def publish_fake_data(self):
        elapsed = time.time() - self.start_time

        # Simulate some nice rolling/pitching values
        roll = 45 * math.sin(elapsed * 0.5)
        pitch = 85 * math.sin(elapsed * 0.7)
        yaw = (elapsed * 20) % 360  # simulate rotating heading

        # Publish euler angles
        euler_msg = Float32MultiArray()
        euler_msg.data = [roll, pitch, yaw]
        self.euler_pub.publish(euler_msg)

        # Publish heading
        heading_msg = Float32()
        heading_msg.data = yaw
        self.heading_pub.publish(heading_msg)

        self.get_logger().info(f"Published R={roll:.1f}, P={pitch:.1f}, Y={yaw:.1f}")

def main(args=None):
    rclpy.init(args=args)
    node = FakeIMUPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
