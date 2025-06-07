import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import time
from math import sqrt, atan2, sin, cos


def normalize_angle(angle):
    return atan2(sin(angle), cos(angle))


class OdomMover(Node):
    def __init__(self):
        super().__init__('odom_mover')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.odom_subscriber = self.create_subscription(
            Odometry, '/odometry/filtered', self.odom_callback, 10)

        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0
        self.odom_received = False

    def odom_callback(self, msg):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y

        # Extract yaw from quaternion
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.current_yaw = atan2(siny_cosp, cosy_cosp)

        self.odom_received = True

    def stop(self):
        twist = Twist()
        self.publisher_.publish(twist)

    def move_forward_by_distance(self, distance, speed=0.2):
        self.get_logger().info(f'Moving forward {distance} m at speed {speed}...')
        
        while not self.odom_received:
            rclpy.spin_once(self)

        # รอให้ตำแหน่งเริ่มเปลี่ยนแปลงเล็กน้อยก่อนเริ่ม
        self.get_logger().info('Waiting for initial position to stabilize...')
        init_x = self.current_x
        init_y = self.current_y

        wait_count = 0
        while wait_count < 50:  # รอสูงสุด 5 วินาที
            rclpy.spin_once(self)
            dx = self.current_x - init_x
            dy = self.current_y - init_y
            if sqrt(dx * dx + dy * dy) > 0.005:  # ขยับอย่างน้อย 0.5 ซม.
                self.get_logger().info(f'Position changed: ({self.current_x:.2f}, {self.current_y:.2f})')
                break
            time.sleep(0.1)
            wait_count += 1

        start_x = self.current_x
        start_y = self.current_y

        twist = Twist()
        twist.linear.x = speed

        while rclpy.ok():
            rclpy.spin_once(self)

            dx = self.current_x - start_x
            dy = self.current_y - start_y
            dist_moved = sqrt(dx * dx + dy * dy)

            if dist_moved >= distance - 0.2:  # หยุดก่อนถึง 5 ซม.
                break

            self.get_logger().info(
                f'Current position: ({self.current_x:.2f}, {self.current_y:.2f}), '
                f'Distance moved: {dist_moved:.2f} m, Target: {distance:.2f} m'
            )
            self.publisher_.publish(twist)

        self.stop()
        self.get_logger().info('Finished moving forward.')

    def turn_to_yaw(self, goal_yaw_rad, angular_speed=0.4):
        self.get_logger().info(
            f'Turning to yaw = {goal_yaw_rad:.3f} rad at angular speed {angular_speed}...')

        while not self.odom_received:
            rclpy.spin_once(self)

        twist = Twist()

        while rclpy.ok():
            rclpy.spin_once(self)
            error = normalize_angle(goal_yaw_rad - self.current_yaw)
            self.get_logger().info(
                f'Current yaw: {self.current_yaw:.3f}, Goal: {goal_yaw_rad:.3f}, Error: {error:.3f}')

            if abs(error) < 0.01:
                break

            twist.angular.z = angular_speed if error > 0 else -angular_speed
            self.publisher_.publish(twist)

        self.stop()
        self.get_logger().info(f'Finished turning to yaw = {goal_yaw_rad:.3f} rad.')


def main(args=None):
    rclpy.init(args=args)
    node = OdomMover()

    distance_per_side = 5.0      # เดินหน้าแต่ละด้านของสี่เหลี่ยม (เมตร)
    forward_speed = 0.5          # ความเร็วเดินหน้า (m/s)
    angular_speed = 0.2          # ความเร็วหมุน (rad/s)
    turn_angle_deg = -89         # มุมหมุนต่อด้าน (องศา)
    square_loops = 1             # จำนวนรอบที่ทำรูปสี่เหลี่ยม
    delay_between_steps = 4      # หน่วงเวลา (วินาที)

    turn_angle_rad = turn_angle_deg * 3.14159265359 / 180.0

    # รอรับ odom แรก
    node.get_logger().info("Waiting for initial odometry...")
    while not node.odom_received:
        rclpy.spin_once(node)
    node.get_logger().info(f"Initial odometry received at ({node.current_x:.2f}, {node.current_y:.2f})")

    current_goal_yaw = node.current_yaw

    for i in range(square_loops):
        node.get_logger().info(f"Square loop iteration: {i + 1}/{square_loops}")

        node.move_forward_by_distance(distance_per_side, speed=forward_speed)
        time.sleep(delay_between_steps)

        current_goal_yaw = normalize_angle(current_goal_yaw + turn_angle_rad)
        # node.turn_to_yaw(current_goal_yaw, angular_speed=angular_speed)
        time.sleep(delay_between_steps)

    node.get_logger().info('Completed all square loops.')
    node.stop()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
