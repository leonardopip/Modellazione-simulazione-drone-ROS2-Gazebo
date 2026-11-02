import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.duration import Duration
from actuator_msgs.msg import Actuators

from primo_controllo.sensori import IMUSensor
from primo_controllo.motori import PublisherVelocity


def main(args=None):
    rclpy.init(args=args)
    velocita = int(input("Inserisci la velocità dei motori: "))

    imu_sensor = IMUSensor()
    motor_velocity = PublisherVelocity(velocita)
    clock = imu_sensor.get_clock()
    primo_fronte = None
    secondo_fronte = None
    try:
        while rclpy.ok():
            rclpy.spin_once(imu_sensor, timeout_sec=0.1)
            rclpy.spin_once(motor_velocity, timeout_sec=0.05)

            if imu_sensor.imu is None:
                continue

            # Access the IMU data
            orientation = imu_sensor.imu.orientation
            angular_velocity = imu_sensor.imu.angular_velocity
            linear_acceleration = imu_sensor.imu.linear_acceleration

            az = linear_acceleration.z
            now = clock.now()



            if az >10.9:
                secondo_fronte = None
                if primo_fronte is None:
                    primo_fronte = now
                if now - primo_fronte > Duration(seconds=3):
                    motor_velocity.vel_msg = Actuators(velocity=[0.0, 0.0, 0.0, 0.0]) #il drone è salito per 5s

                    print("Il drone è salito per 3 secondi, motori spenti.")
                    secondo_fronte = now

            if  secondo_fronte is not None:
                if now - secondo_fronte > Duration(seconds=5):
                    print("Il drone è sceso per 5 secondi, motori accesi.")
                    print("Il drone è in volo, motori accesi.", velocita)
                    motor_velocity.vel_msg = Actuators(velocity=[velocita, velocita, velocita, velocita]) #il drone non è salito per 5s
                    primo_fronte = None
                

                
    except KeyboardInterrupt:
        pass    

    finally:
        if rclpy.ok():
            rclpy.shutdown()
        

