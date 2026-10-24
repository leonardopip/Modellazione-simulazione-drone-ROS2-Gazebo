import rclpy
from rclpy.node import Node

from actuator_msgs.msg import Actuators


class PublisherVelocity(Node):

    def __init__(self, velocita):
        super().__init__('velocity_publisher')
        self.publisher_ = self.create_publisher(
            Actuators,
            '/X3/gazebo/command/motor_speed',
            10
        )
        self.vel_msg = velocita
        self.create_timer(0.1, self.timer_callback)  

    def timer_callback(self):
        self.publisher_.publish(self.vel_msg)