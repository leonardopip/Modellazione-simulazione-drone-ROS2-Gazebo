import rclpy
from rclpy.executors import SingleThreadedExecutor
from actuator_msgs.msg import Actuators

from pad_control.pad_sub import Pad_Sub
from primo_controllo.motori import PublisherVelocity

VEL_MAX = 800.0

THRUST_MAX = 0.9   # lascia margine ai comandi di assetto
ROLL_MAX   = 0.1  # frazione di VEL_MAX
PITCH_MAX  = 0.1
YAW_MAX    = 0.05

def mixer(spinta, pitch, roll, rot_dx, rot_sx):
    """Ingressi pad: spinta, pitch, roll in [-1,1]; rot in {0,1}. Uscita: rad/s."""
    if spinta <= 0:
        return [0.0] * 4

    t = spinta * THRUST_MAX
    r = roll * ROLL_MAX      # >0 = destra
    p = pitch * PITCH_MAX    # >0 = avanti
    y_sx = rot_sx * YAW_MAX
    y_dx = rot_dx * YAW_MAX

    # layout: 0 dietro-dx, 1 davanti-sx, 2 dietro-sx, 3 davanti-dx
    m = [
        t - r + p - y_sx,   # 0
        t + r - p - y_sx,   # 1
        t + r + p - y_dx,   # 2
        t - r - p - y_dx,   # 3
    ]
    return [max(0.0, min(1.0, v))* VEL_MAX for v in m]

def main(args=None):
    rclpy.init(args=args)

    motor_velocity = PublisherVelocity(0.0)
    comandi = Pad_Sub()

    executor = SingleThreadedExecutor()
    executor.add_node(comandi)
    executor.add_node(motor_velocity)

    print("Controllo attivo")
    try:
        while rclpy.ok():
            executor.spin_once(timeout_sec=0.02)

            valori = (comandi.spinta, comandi.pitch, comandi.roll,
                      comandi.rotazioneDx, comandi.rotazioneSx)
            if any(v is None for v in valori):
                continue

            motor_velocity.vel_msg = Actuators(velocity=mixer(*valori))
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        comandi.destroy_node()
        motor_velocity.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()