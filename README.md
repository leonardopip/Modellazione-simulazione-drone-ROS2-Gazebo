# Modello UAV x3
Il modello del drone è stato scaricato dalla libreria Fuel di gazebo
```bibtex
@online{GazeboFuel-OpenRobotics-X3-UAV,
	title={X3 UAV},
	organization={Open Robotics},
	date={2023},
	month={September},
	day={3},
	author={OpenRobotics},
	url={https://app.gazebosim.org/OpenRobotics/fuel/models/X3%20UAV},
}
```


All'interno non erano presenti i plug in dei motori li ho copiati da GitHub [X3 UAV Model on GitHub](https://github.com/gazebosim/gz-sim/blob/main/examples/worlds/quadcopter.sdf).

rotore 0 e 1 lavorano ccw, rotore 2 e 3 clock wise


Per l'implementazione dell'odometria e IMU ho inserito dei plugin all'interno del modello del drone

```
<plugin filename="gz-sim-odometry-publisher-system"
        name="gz::sim::systems::OdometryPublisher">
        <odom_frame>odom</odom_frame>
        <robot_base_frame>X3/base_link</robot_base_frame>
        <dimensions>3</dimensions>
</plugin>
      
<sensor name="imu_sensor" type="imu">
      <always_on>1</always_on>
      <update_rate>100</update_rate>
      <topic>imu</topic>
```


# Pacchetti ROS
Il primo pacchetto sviluppato è il primo_controllo, il programma OIV.py (Odometry-IMU-velocity) usa due nodi provenienti da sensori.py per un subscriber(Odometry) e il nodo di publisher per le velocità dei rotori da motori.py

In OIV.py ho creato un semplice codice che inizialmente chiede una velocità dei rotori e quando arriva ad una determinata quota li spegne.

## Pad_control
il pacchetto pad control permette di controllare il drone modificando le velocità dei rotori. Utilizzando la l'analogico di sinistra di un controller un movimento su e giu rispettivamente aumenta e diminuisce le velocità dei motori, una spinta sinistra-destra invece modifica le velocita dei rotori per permettere di inclinare e spostare sul piano orizzontale il drone

# Start
Per lanciare la simulazionoe ROS-GAZEBO-RVIZ
```
ros2 launch launch/sim.launch.py
```




