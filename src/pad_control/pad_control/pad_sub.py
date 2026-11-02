import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy


class Pad_Sub(Node):
    def __init__(self):
        super().__init__('pad_sub')
        self.subscription = self.create_subscription(Joy,'joy',self.joy_callback,10)
        self.subscription  # prevent unused variable warning

    def joy_callback(self, msg):
        header = msg.header
        axes = msg.axes
        buttons = msg.buttons

        self.spinta_verticale = None
        self.spinta_orizzontale = None
        
        if axes is not None and buttons is not None:
            self.spinta_verticale = axes[3]
            self.spinta_orizzontale = axes[2]
            




        
