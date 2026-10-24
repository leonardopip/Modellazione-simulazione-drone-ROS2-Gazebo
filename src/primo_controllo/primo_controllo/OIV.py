import rclpy
from rclpy.executors import SingleThreadedExecutor
from actuator_msgs.msg import Actuators

from primo_controllo.sensori import OdometrySensor
from primo_controllo.motori import PublisherVelocity

def main(args=None):
    rclpy.init(args=args)
    velocita = Actuators(velocity=[int(input("R1: ")), int(input("R2: ")), int(input("R3: ")), int(input("R4: "))])

    motor_velocity = PublisherVelocity(velocita)
    odom_sensor = OdometrySensor()
    stopped = False
    
    try:
        while rclpy.ok():
            rclpy.spin_once(odom_sensor, timeout_sec=0.1)
            rclpy.spin_once(motor_velocity, timeout_sec=0.05)

            if odom_sensor.odom is None or stopped:
                continue

            z = odom_sensor.odom.pose.pose.position.z
            if z > 5:
                print("Z value exceeded 5. Stopping Motors.")
                motor_velocity.vel_msg = Actuators(velocity=[0.0, 0.0, 0.0, 0.0])
                stopped = True

    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()
            

    rclpy.shutdown()
if __name__ == '__main__':
    main()