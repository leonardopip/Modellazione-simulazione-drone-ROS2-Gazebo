import rclpy
from rclpy.executors import SingleThreadedExecutor
from actuator_msgs.msg import Actuators

from pad_control.pad_sub import Pad_Sub
from primo_controllo.motori import PublisherVelocity
from primo_controllo.motori import VelocityStatus

VEL_MAX = 800.0
VEL_MAX_WORK = 700.0
START_VEL = 500.0
STEP = 10.0


def flight(omega_obs,vel_comandata,spinta, pitch, roll, rot_dx, rot_sx, accensione, spegnimento,FRONTE):
    if accensione == 1 and FRONTE == 0:
        FRONTE = 1
        vel_comandata = [START_VEL] * 4
        return vel_comandata,FRONTE
        

    if spegnimento == 1 and FRONTE == 1:
        FRONTE = 0
        vel_comandata = [0.0] * 4
        return vel_comandata,FRONTE
    
    # un passo per ciclo a partire dalla velocità attuale, niente while
    if spinta > 0 and FRONTE == 1:
        vel_comandata = [min(v + STEP, VEL_MAX_WORK) for v in omega_obs]
        return vel_comandata,FRONTE
    elif spinta < 0 and FRONTE == 1:
        vel_comandata = [max(v - STEP, 0.0) for v in omega_obs]
        return vel_comandata,FRONTE
    return vel_comandata, FRONTE   # spinta == 0: mantieni


def main(args=None):
    rclpy.init(args=args)
    motor_velocity = PublisherVelocity(0.0)
    motor_status = VelocityStatus()
    comandi = Pad_Sub()

    executor = SingleThreadedExecutor()
    executor.add_node(comandi)
    executor.add_node(motor_velocity)
    executor.add_node(motor_status)   # lo stesso nodo che poi leggi

    print("Controllo attivo")
    try:
        FRONTE = 0
        vel_comandata = [0.0] * 4
        while rclpy.ok():
            executor.spin_once(timeout_sec=0.02)

            valori = (comandi.spinta, comandi.pitch, comandi.roll,
                      comandi.rotazioneDx, comandi.rotazioneSx,
                      comandi.accensione, comandi.spegnimento)


            if any(v is None for v in valori):
                continue

            vel_comandata, FRONTE = flight(list(motor_status.vel),vel_comandata,*valori,FRONTE)

            print(vel_comandata,motor_status.vel)


            motor_velocity.vel_msg = Actuators(velocity=vel_comandata)
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        comandi.destroy_node()
        motor_velocity.destroy_node()
        motor_status.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()