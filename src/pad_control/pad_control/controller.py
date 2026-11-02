
import rclpy
import os
import xml.etree.ElementTree as ET



from rclpy.node import Node
from actuator_msgs.msg import Actuators

from pad_control.pad_sub import Pad_Sub
from primo_controllo.motori import PublisherVelocity


def VelocitaRotori(model_path):
    root = ET.parse(model_path).getroot()
    return max(float(v.text) for v in root.iter("maxRotVelocity"))

def main(args=None):
    rclpy.init(args=args)

    #model_path = os.path.expanduser("~/gz_ws/models/x3/model.sdf")  # adatta al tuo percorso
    velocita_massima = 800
    #if not velocita_massima:
    #    raise RuntimeError(f"Nessun maxRotVelocity trovato in {model_path}")
    n_rotori = 4

    motor_velocity = PublisherVelocity(0.0)
    comandi = Pad_Sub()
    stopped = False
    
    
    try:
        while rclpy.ok():
            rclpy.spin_once(comandi, timeout_sec=0.05)
            rclpy.spin_once(motor_velocity, timeout_sec=0.05)

            if comandi.spinta_verticale is not None and comandi.spinta_orizzontale is not None:
                if comandi.spinta_verticale > 0:
                    velocita = comandi.spinta_verticale * velocita_massima
                    motor_velocity.vel_msg = Actuators(velocity=[velocita,velocita,velocita,velocita] )

                if comandi.spinta_orizzontale > 0: #spostamento a destra
                    motor_velocity.vel_msg = Actuators(velocity=[velocita*(1-(comandi.spinta_orizzontale)/100), velocita, velocita, velocita*(1-(comandi.spinta_orizzontale)/100)])
                elif comandi.spinta_orizzontale < 0: #spostamento a sinistra
                    motor_velocity.vel_msg = Actuators(velocity=[velocita, velocita*(1+(comandi.spinta_orizzontale)/100), velocita*(1+(comandi.spinta_orizzontale)/100), velocita])
            else:
                motor_velocity.vel_msg = Actuators(velocity=[0.0,0.0,0.0,0.0])
            
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()
            

    rclpy.shutdown()
if __name__ == '__main__':
    main()