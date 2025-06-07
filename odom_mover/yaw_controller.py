import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf_transformations import euler_from_quaternion
import math


def normalize_angle(angle):
    """ Wrap angle to [-pi, pi] """
    return math.atan2(math.sin(angle), math.cos(angle))


class YawController(Node):
    def __init__(self):
        super().__init__('yaw_controller')

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)

        self.target_yaw_deg = 90.0  # 🧭 เปลี่ยนตรงนี้ได้ (เช่น 45, -90, 180, -179)
        self.target_yaw = math.radians(self.target_yaw_deg)

        self.yaw_now = None
        self.tolerance = math.radians(2.0)  # 2 degrees
        self.angular_speed = 0.3  # rad/s

        self.timer = self.create_timer(0.1, self.control_loop)
        self.reached = False

    def odom_callback(self, msg):
        q = msg.pose.pose.orientation
        _, _, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
        self.yaw_now = yaw

    def control_loop(self):
        if self.yaw_now is None or self.reached:
            return

        error = normalize_angle(self.target_yaw - self.yaw_now)

        twist = Twist()

        if abs(error) > self.tolerance:
            twist.angular.z = self.angular_speed if error > 0 else -self.angular_speed
            direction = 'CW' if twist.angular.z < 0 else 'CCW'
            self.get_logger().info(
                f"Rotating {direction} | Yaw now: {
                    math.degrees(
                        self.yaw_now):.2f}° → Target: {
                    self.target_yaw_deg:.2f}°")
        else:
            twist.angular.z = 0.0
            self.reached = True
            self.get_logger().info(f"✅ Reached yaw goal: {self.target_yaw_deg:.2f}°")

        self.cmd_pub.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    node = YawController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
