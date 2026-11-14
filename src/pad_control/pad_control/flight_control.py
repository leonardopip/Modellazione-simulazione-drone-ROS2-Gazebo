import rclpy
import time

from rclpy.executors import SingleThreadedExecutor
from actuator_msgs.msg import Actuators

from pad_control.pad_sub import Pad_Sub
from primo_controllo.motori import PublisherVelocity

VEL_MAX = 800.0
VEL_MAX_WORK = 700.0
START_VEL = 500.0
RATE = 100.0        # rad/s al secondo con stick a fondo (spinta)
DEADZONE = 0.1

# Ampiezza massima delle manovre in rad/s (con stick a fondo)
K_PITCH = 50.0
K_ROLL = 50.0
K_YAW = 30.0

#              m0   m1   m2   m3
PITCH_MIX = [-1.0, +1.0, -1.0, +1.0]
ROLL_MIX  = [-1.0, +1.0, +1.0, -1.0]
YAW_MIX   = [-1.0, -1.0, +1.0, +1.0]


def clamp(x, lo, hi):
    return min(max(x, lo), hi)


def deadzone(x):
    return 0.0 if abs(x) < DEADZONE else x


def flight(vel_base, spinta, pitch, roll, rot_dx, rot_sx,
           accensione, spegnimento, fronte, dt):
    yaw = rot_sx - rot_dx   # +1 antiorario, -1 orario, 0 fermo

    if accensione == 1 and fronte == 0:
        fronte = 1
        vel_base = START_VEL

    if spegnimento == 1 and fronte == 1:
        fronte = 0
        vel_base = 0.0

    if fronte == 0:
        return vel_base, [0.0] * 4, fronte

    # Spinta comune: integratore
    vel_base = clamp(vel_base + RATE * dt * spinta, 0.0, VEL_MAX_WORK)

    # Manovre: offset differenziali, non accumulati
    cmd = [
        clamp(vel_base
              + K_PITCH * pitch * PITCH_MIX[i]
              + K_ROLL * roll * ROLL_MIX[i]
              + K_YAW * yaw * YAW_MIX[i],
              0.0, VEL_MAX)
        for i in range(4)
    ]
    return vel_base, cmd, fronte


def main(args=None):
    rclpy.init(args=args)
    motor_velocity = PublisherVelocity(0.0)
    comandi = Pad_Sub()

    executor = SingleThreadedExecutor()
    executor.add_node(comandi)
    executor.add_node(motor_velocity)

    print("Controllo attivo")
    try:
        FRONTE = 0
        vel_base = 0.0
        t_prev = time.monotonic()
        while rclpy.ok():
            executor.spin_once(timeout_sec=0.02)

            t_now = time.monotonic()
            dt = t_now - t_prev
            t_prev = t_now

            valori = (comandi.spinta, comandi.pitch, comandi.roll,
                      comandi.rotazioneDx, comandi.rotazioneSx,
                      comandi.accensione, comandi.spegnimento)

            if any(v is None for v in valori):
                continue

            vel_base, cmd, FRONTE = flight(vel_base, *valori, FRONTE, dt)
            motor_velocity.vel_msg = Actuators(velocity=cmd)
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