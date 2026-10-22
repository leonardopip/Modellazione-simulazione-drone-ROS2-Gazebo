import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry

class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            Odometry,
            '/x3/odometry',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info(f'Posizione X: {msg.pose.pose.position.x}, Y: {msg.pose.pose.position.y}, Z: {msg.pose.pose.position.z}, Orientation X: {msg.pose.pose.orientation.x}, Y: {msg.pose.pose.orientation.y}, Z: {msg.pose.pose.orientation.z}, W: {msg.pose.pose.orientation.w}')


def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()

    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()