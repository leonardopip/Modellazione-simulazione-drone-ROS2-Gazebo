import rclpy
import numpy as np
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import Float64MultiArray

DEADZONE = 0.05
JOY_TIMEOUT = 0.5  # secondi


def deadzone(x, dz=DEADZONE):
    return 0.0 if abs(x) < dz else x


class LowPassFilter:
    def __init__(self, alpha):
        self.alpha = alpha
        self.prev_value = None

    def filter(self, value):
        if self.prev_value is None:
            self.prev_value = value
        else:
            self.prev_value = self.alpha * value + (1 - self.alpha) * self.prev_value
        return self.prev_value


class Pad_Sub(Node):
    def __init__(self):
        super().__init__('pad_sub')

        self.thrust = 0.0
        self.pitch = 0.0
        self.roll = 0.0
        self.rotazioneDx = 0.0
        self.rotazioneSx = 0.0
        self.accensione = 0
        self.spegnimento = 0
        self.last_msg_time = None

        self.filter_thrust = LowPassFilter(0.2)
        self.filter_pitch = LowPassFilter(0.2)
        self.filter_roll = LowPassFilter(0.2)

        self.subscription = self.create_subscription(
            Joy, '/joy', self.joy_callback, 10)

        

        self.ComandiPub = self.create_publisher(Float64MultiArray,'/Comandi',10)
        self.timer = self.create_timer(0.01, self.ComandiMatlab)

        self.get_logger().info('Pad Node avviato')


    def joy_callback(self, msg):
        if len(msg.axes) > 4:
            self.thrust = self.filter_thrust.filter(deadzone(msg.axes[4]))
            self.pitch = self.filter_pitch.filter(deadzone(msg.axes[1]))
            # segno invertito: in ROS joy, sinistra = positivo
            self.roll = self.filter_roll.filter(deadzone(-msg.axes[0]))

        if len(msg.buttons) > 5:
            self.rotazioneDx = float(msg.buttons[5])
            self.rotazioneSx = float(msg.buttons[4])
            self.accensione = float(msg.buttons[0])
            self.spegnimento = float(msg.buttons[1])

        self.last_msg_time = self.get_clock().now()

        self.get_logger().info(
            f'Thrust: {self.thrust:.3f}, Pitch: {self.pitch:.3f}, '
            f'Roll: {self.roll:.3f}, DX: {self.rotazioneDx}, SX: {self.rotazioneSx}',
            throttle_duration_sec=1.0)

    def pronto(self):
        """True se è arrivato almeno un messaggio e non è scaduto."""
        if self.last_msg_time is None:
            return False
        età = (self.get_clock().now() - self.last_msg_time).nanoseconds * 1e-9
        return età < JOY_TIMEOUT

    def ComandiMatlab(self):
        comandi = [self.thrust, 
                    self.pitch,
                    self.roll,
                    self.rotazioneDx,
                    self.rotazioneSx,
                    self.accensione,
                    self.spegnimento]
        self.ComandiPub.publish(Float64MultiArray(data=comandi))
        


def main(args=None):
    rclpy.init(args=args)
    node = Pad_Sub()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()