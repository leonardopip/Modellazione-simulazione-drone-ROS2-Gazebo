import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu

class OdometrySensor(Node):
    def __init__(self):
        super().__init__('odom_sensor')
        self.odom = None
        self.create_subscription(Odometry, '/x3/odometry', self.odom_callback, 10)

    def odom_callback(self, msg):
        self.odom = msg

class IMUSensor(Node):
    def __init__(self):
        super().__init__('imu_sensor')
        self.imu = None
        self.create_subscription(Imu, '/x3/imu', self.imu_callback, 10)

    def imu_callback(self, msg):
        self.imu = msg