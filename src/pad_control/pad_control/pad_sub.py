import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy


class Pad_Sub(Node):
    def __init__(self):
        super().__init__('pad_sub')
        self.spinta_verticale = None
        self.spinta_orizzontale = None

        self.subscription = self.create_subscription(
            Joy, '/joy', self.joy_callback, 10)

    def joy_callback(self, msg):
        if msg.axes and msg.buttons:
            self.spinta_verticale = msg.axes[1]
            self.spinta_orizzontale = msg.axes[0]            




        
