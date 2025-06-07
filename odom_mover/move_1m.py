import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from math import sqrt


class OdomBasedMover(Node):
    def __init__(self):
        super().__init__('odom_mover')
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.subscription = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.odom_received = False
        self.start_x = 0.0
        self.start_y = 0.0
        self.current_x = 0.0
        self.current_y = 0.0

    def odom_callback(self, msg):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        if not self.odom_received:
            self.start_x = self.current_x
            self.start_y = self.current_y
            self.odom_received = True

    def move_forward_by_distance(self, distance=1.0, speed=0.2):
        twist = Twist()
        twist.linear.x = speed
        self.get_logger().info(f'Moving forward {distance} m using /odom...')

        # wait for odometry data
        while not self.odom_received:
            rclpy.spin_once(self)

        self.publisher_.publish(twist)
        while rclpy.ok():
            rclpy.spin_once(self)
            dist_moved = sqrt((self.current_x - self.start_x) ** 2 +
                              (self.current_y - self.start_y) ** 2)
            self.get_logger().info(f'Current position: ({self.current_x:.2f}, {self.current_y:.2f}), '
                                   f'Distance moved: {dist_moved:.2f} m')
            if dist_moved >= distance:
                self.get_logger().info(f'Reached {dist_moved:.2f} m, stopping.')
                break
            self.publisher_.publish(twist)

        self.stop()

    def stop(self):
        twist = Twist()
        self.publisher_.publish(twist)
        rclpy.spin_once(self)
        self.get_logger().info("Robot stopped.")


def main():
    rclpy.init()
    node = OdomBasedMover()
    node.move_forward_by_distance(distance=1.0, speed=0.2)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
