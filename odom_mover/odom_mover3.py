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

        # Yaw from Quaternion
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.current_yaw = atan2(siny_cosp, cosy_cosp)

        self.odom_received = True

    def stop(self):
        twist = Twist()
        self.publisher_.publish(twist)

    def move_forward_by_distance(self, target_distance, speed=1.0):
        self.get_logger().info(f'Moving forward {target_distance} m with PID control...')

        while not self.odom_received:
            rclpy.spin_once(self)

        start_x = self.current_x
        start_y = self.current_y

        # tune คร่าวๆ
        # 0.5 m/s use P=1.0, I=0, D=0.3 
        # 1.0 m/s use P=0.9, I=0.01, D=0.4

        # PID parameters
        Kp = 0.9
        Ki = 0.01
        Kd = 0.2

        prev_error = 0.0
        integral = 0.0
        dt = 0.02  # control update period (s)

        twist = Twist()

        while rclpy.ok():
            rclpy.spin_once(self)

            dx = self.current_x - start_x
            dy = self.current_y - start_y
            distance_moved = sqrt(dx * dx + dy * dy)

            error = target_distance - distance_moved
            if error <= 0.01:  # tolerance 1 cm
                break

            # PID calculations
            integral += error * dt
            derivative = (error - prev_error) / dt
            output = Kp * error + Ki * integral + Kd * derivative
            prev_error = error

            # Clamp output speed to [0, max_speed]
            speed = max(0.0, min(speed, output))

            twist.linear.x = speed
            self.publisher_.publish(twist)

            self.get_logger().info(
                f'Distance moved: {distance_moved:.2f} m, Error: {error:.2f}, Speed: {speed:.2f}'
            )

            time.sleep(dt)

        self.stop()
        self.get_logger().info('Finished moving forward (PID).')

    # ยังไม่ได้ใส่ PID
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

    # adjust parameters for test
    distance_per_side = 5.0      # distance target (m)
    forward_speed = 1.0          # forward speed (m/s)
    angular_speed = 0.2          # angular speed (rad/s)
    turn_angle_deg = -90         # Rotation angle per side (degrees)
    square_loops = 1             # Number of rounds
    delay_between_steps = 4      # Delays (s)

    turn_angle_rad = turn_angle_deg * 3.14159265359 / 180.0

    # Waiting to receive the first odom
    node.get_logger().info("Waiting for initial odometry...")
    while not node.odom_received:
        rclpy.spin_once(node)
    node.get_logger().info(f"Initial odometry received at ({node.current_x:.2f}, {node.current_y:.2f})")

    current_goal_yaw = node.current_yaw

    for i in range(square_loops):
        node.get_logger().info(f"Square loop iteration: {i + 1}/{square_loops}")

        node.move_forward_by_distance(distance_per_side, speed=forward_speed)
        #time.sleep(delay_between_steps)

        # current_goal_yaw = normalize_angle(current_goal_yaw + turn_angle_rad)
        # node.turn_to_yaw(current_goal_yaw, angular_speed=angular_speed)
        #time.sleep(delay_between_steps)

    node.get_logger().info('Completed all sequence loops.')
    node.stop()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
