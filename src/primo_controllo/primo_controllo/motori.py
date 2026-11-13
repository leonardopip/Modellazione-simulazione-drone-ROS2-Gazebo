from rclpy.node import Node

from std_msgs.msg import Float64
from actuator_msgs.msg import Actuators


class PublisherVelocity(Node):
    def __init__(self, velocita):
        super().__init__('velocity_publisher')
        self.publisher_ = self.create_publisher(
            Actuators,
            '/X3/gazebo/command/motor_speed',
            10
        )
        self.vel_msg = Actuators(velocity=[velocita] * 4)
        self.create_timer(0.1, self.timer_callback)

    def timer_callback(self):
        self.publisher_.publish(self.vel_msg)


class VelocityStatus(Node):
    def __init__(self):
        super().__init__('velocity_status')
        self.vel = [0.0] * 4   # velocità attuali dei 4 rotori

        for i in range(4):
            self.create_subscription(
                Float64, f'/X3/motor_speed/cmd{i}',
                lambda msg, i=i: self.OmegaMotor_cb(msg, i), 10)

    def OmegaMotor_cb(self, msg, i):
        self.vel[i] = msg.data