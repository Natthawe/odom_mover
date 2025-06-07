import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf_transformations import euler_from_quaternion
import math


class YawController(Node):
    def __init__(self):
        super().__init__('yaw_controller')

        # Publisher: ส่งคำสั่งไปที่ cmd_vel
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Subscriber: รับค่า odometry
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)

        self.timer = self.create_timer(0.05, self.control_loop)

        # State
        self.current_yaw = 0.0
        self.goal_yaw = math.radians(45)  # 🔄 เป้าหมายหมุน 90 องศาไปทางขวา
        self.reached = False

    def odom_callback(self, msg):
        # แปลง quaternion → euler → yaw
        q = msg.pose.pose.orientation
        _, _, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
        self.current_yaw = yaw

    def shortest_angular_distance(self, current, target):
        diff = (target - current + math.pi) % (2 * math.pi) - math.pi
        return diff

    def control_loop(self):
        if self.reached:
            return

        yaw_diff = self.shortest_angular_distance(self.current_yaw, self.goal_yaw)

        cmd = Twist()

        # เช็คว่าหมุนถึงเป้าหมายหรือยัง (ภายใน 2 องศา)
        if abs(yaw_diff) < math.radians(2):
            self.get_logger().info("✅ Reached goal yaw")
            self.reached = True
            cmd.angular.z = 0.0
        else:
            # สั่งหมุน (ปรับความเร็วตามระยะห่างมุม)
            cmd.angular.z = 0.5 * yaw_diff  # ปรับ gain ได้

        self.cmd_pub.publish(cmd)


def main():
    rclpy.init()
    node = YawController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
