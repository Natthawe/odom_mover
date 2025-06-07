import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from math import sqrt, pi
from transforms3d.euler import quat2euler  # ใช้แทน tf_transformations
import time


class OdomBasedMover(Node):
    def __init__(self):
        super().__init__('odom_mover')
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.subscription = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.odom_received = False
        self.start_x = 0.0
        self.start_y = 0.0
        self.start_yaw = 0.0
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0

    def odom_callback(self, msg):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        # quaternion to euler
        (_, _, yaw) = quat2euler([q.w, q.x, q.y, q.z])  # note: [w, x, y, z] order
        self.current_yaw = yaw
        if not self.odom_received:
            self.start_x = self.current_x
            self.start_y = self.current_y
            self.start_yaw = self.current_yaw
            self.odom_received = True

    def move_forward_by_distance(self, distance=1.0, speed=0.2):
        twist = Twist()
        twist.linear.x = speed
        self.get_logger().info(f'Moving forward {distance} m using /odom...')

        while not self.odom_received:
            rclpy.spin_once(self)

        self.publisher_.publish(twist)
        while rclpy.ok():
            rclpy.spin_once(self)
            dist_moved = sqrt((self.current_x - self.start_x) ** 2 +
                              (self.current_y - self.start_y) ** 2)
            if dist_moved >= distance:
                self.get_logger().info(f'Reached {dist_moved:.2f} m, stopping.')
                break
            self.publisher_.publish(twist)
            self.get_logger().info(f'Current position: ({self.current_x:.2f}, {self.current_y:.2f}), '
                                   f'Distance moved: {dist_moved:.2f} m, '
                                   f'Current yaw: {self.current_yaw:.2f} rad, '
                                   f'Start yaw: {self.start_yaw:.2f} rad')
        self.stop()

    def turn_right_by_angle(self, angle_deg=90, angular_speed=0.5):
        target_angle = angle_deg * pi / 180.0
        twist = Twist()
        twist.angular.z = -abs(angular_speed)
        self.get_logger().info(f'Turning right {angle_deg} degrees using /odom...')

        while not self.odom_received:
            rclpy.spin_once(self)

        self.publisher_.publish(twist)
        while rclpy.ok():
            rclpy.spin_once(self)
            # คำนวณการหมุนจริงแบบ wrap-around
            delta_yaw = (self.start_yaw - self.current_yaw) % (2 * pi)
            if delta_yaw > pi:
                delta_yaw -= 2 * pi  # ให้ค่าระหว่าง -pi ถึง pi
            if abs(delta_yaw) >= target_angle:
                self.get_logger().info(f'Rotated {angle_deg} degrees, stopping.')
                break
            self.publisher_.publish(twist)
            self.get_logger().info(f'Current yaw: {self.current_yaw:.2f} rad, '
                                   f'Target angle: {target_angle:.2f} rad, '
                                   f'Delta yaw: {delta_yaw:.2f} rad')
        self.stop()

    def stop(self):
        twist = Twist()
        self.publisher_.publish(twist)
        rclpy.spin_once(self)
        self.get_logger().info("Robot stopped.")
        time.sleep(1)


def main():
    rclpy.init()
    node = OdomBasedMover()
    node.move_forward_by_distance(distance=1.0, speed=0.5)
    node.turn_right_by_angle(angle_deg=90, angular_speed=0.5)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
