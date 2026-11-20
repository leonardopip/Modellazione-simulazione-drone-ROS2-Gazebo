import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from actuator_msgs.msg import Actuators


class PublisherVelocity(Node):
    def __init__(self, velocita=0.0):
        super().__init__('velocity_publisher')

        # uscita verso Gazebo (invariata)
        self.publisher_ = self.create_publisher(
            Actuators, '/X3/gazebo/command/motor_speed', 10)

        # ingresso dal controllore (nuovo)
        self.sub_ = self.create_subscription(
            Float64MultiArray, '/controllore/motor_speed',
            self.cmd_callback, 10)

        self.vel_msg = Actuators(velocity=[float(velocita)] * 4)
        self.last_cmd = self.get_clock().now()
        self.create_timer(0.01, self.timer_callback)   # 100 Hz

    def cmd_callback(self, msg):
        if len(msg.data) != 4:
            self.get_logger().warn(f'Attesi 4 valori, ricevuti {len(msg.data)}')
            return
        self.vel_msg = Actuators(velocity=[float(v) for v in msg.data])
        self.last_cmd = self.get_clock().now()

    def timer_callback(self):
        # watchdog: se il controllore tace per 0.5 s, fermo i motori
        dt = (self.get_clock().now() - self.last_cmd).nanoseconds * 1e-9
        if dt > 0.5:
            self.vel_msg = Actuators(velocity=[0.0] * 4)
        self.publisher_.publish(self.vel_msg)


def main():
    rclpy.init()
    node = PublisherVelocity()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()