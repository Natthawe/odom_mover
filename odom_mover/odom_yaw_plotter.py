import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from tf_transformations import euler_from_quaternion
from std_msgs.msg import Float32

import matplotlib.pyplot as plt
import matplotlib.animation as animation
import math
from collections import deque


class OdomYawPlotter(Node):
    def __init__(self):
        super().__init__('odom_yaw_plotter')

        # Subscribe odom
        self.subscription = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )

        # Optional: Publish yaw to topic
        self.yaw_pub = self.create_publisher(Float32, '/yaw', 10)

        # For plot
        self.yaw_history = deque(maxlen=200)
        self.time_history = deque(maxlen=200)
        self.start_time = self.get_clock().now().nanoseconds / 1e9

        # Start animation
        self.fig, self.ax = plt.subplots()
        self.ani = animation.FuncAnimation(
            self.fig, self.update_plot, interval=100
        )
        plt.xlabel('Time (s)')
        plt.ylabel('Yaw (deg)')
        plt.title('Yaw over Time')
        plt.grid(True)
        plt.tight_layout()

    def odom_callback(self, msg):
        q = msg.pose.pose.orientation
        quaternion = [q.x, q.y, q.z, q.w]
        _, _, yaw = euler_from_quaternion(quaternion)

        yaw_deg = math.degrees(yaw)

        # Save data for plot
        current_time = self.get_clock().now().nanoseconds / 1e9
        elapsed_time = current_time - self.start_time
        self.time_history.append(elapsed_time)
        self.yaw_history.append(yaw_deg)

        # Publish to /yaw
        self.yaw_pub.publish(Float32(data=yaw_deg))

    def update_plot(self, frame):
        self.ax.clear()
        self.ax.plot(self.time_history, self.yaw_history, label='Yaw (deg)')
        self.ax.set_ylim(-180, 180)
        self.ax.set_xlabel('Time (s)')
        self.ax.set_ylabel('Yaw (deg)')
        self.ax.set_title('Yaw over Time')
        self.ax.grid(True)
        self.ax.legend()


def main(args=None):
    rclpy.init(args=args)
    node = OdomYawPlotter()

    try:
        plt.show()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
