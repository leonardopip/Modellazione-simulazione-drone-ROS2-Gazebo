import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry

class OdometrySensor(Node):
    def __init__(self):
        super().__init__('odom_sensor')
        self.odom = None
        self.create_subscription(Odometry, '/x3/odometry', self.odom_callback, 10)

    def odom_callback(self, msg):
        self.odom = msg

        