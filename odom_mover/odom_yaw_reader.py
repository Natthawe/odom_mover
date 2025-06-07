import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from tf_transformations import euler_from_quaternion
import math


class OdomYawReader(Node):
    def __init__(self):
        super().__init__('odom_yaw_reader')

        # Subscribe to /odom
        self.subscription = self.create_subscription(
            Odometry,
            '/odometry/filtered',
            self.odom_callback,
            10
        )

    def odom_callback(self, msg):
        # ดึง orientation จาก /odom (quaternion)
        orientation_q = msg.pose.pose.orientation
        quaternion = [
            orientation_q.x,
            orientation_q.y,
            orientation_q.z,
            orientation_q.w
        ]

        # แปลง quaternion → roll, pitch, yaw
        roll, pitch, yaw = euler_from_quaternion(quaternion)

        # แปลง yaw จาก radian → degree (ถ้าต้องการ)
        yaw_deg = math.degrees(yaw)

        self.get_logger().info(f"Yaw (radian): {yaw:.4f}, Yaw (degree): {yaw_deg:.2f}°")


def main(args=None):
    rclpy.init(args=args)
    node = OdomYawReader()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
