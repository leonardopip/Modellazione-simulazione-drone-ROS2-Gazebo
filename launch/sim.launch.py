import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, SetEnvironmentVariable
from launch_ros.actions import Node


def generate_launch_description():
    # Radice del workspace: cartella sopra "launch/"
    ws_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

    models_path = os.path.join(ws_dir, 'models')
    worlds_path = os.path.join(ws_dir, 'worlds')
    world_file = os.path.join(worlds_path, 'wind_farm_x3.sdf')
    bridge_config = os.path.join(ws_dir, 'config', 'ros_gz_bridge.yaml')
    rviz_config = os.path.join(ws_dir, 'config', 'x3.rviz')
    sdf_file = os.path.join(models_path, 'x3', 'model.sdf')

    # Aggiunge i modelli ai percorsi già presenti (quelli di ROS)
    existing = os.environ.get('GZ_SIM_RESOURCE_PATH', '')
    resource_path = f"{models_path}:{worlds_path}"
    if existing:
        resource_path = f"{resource_path}:{existing}"
    
    
    with open(sdf_file, 'r') as infp:
        robot_desc = infp.read()
    
    set_env = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=resource_path
    )

    gz_sim = ExecuteProcess(
        cmd=['gz', 'sim', '-r','-v', '4', world_file], 
        output='screen'
    ) 
    #-r per far partire la simulazione in modalità "paused"

    #finds all of the non-fixed joints and publishes a JointState message with all those joints defined.
    #joint_state_publisher_gui = Node(
    #    package='joint_state_publisher_gui',
    #    executable='joint_state_publisher_gui',
    #    name='joint_state_publisher_gui',
    #    arguments=[sdf_file],
    #    output=['screen']
    #)

    

    #takes the description and joint angles of the robot as inputs and publishes the 3D poses of the robot links, using a kinematic tree model of the robot
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='both',
        parameters=[
            {'use_sim_time': True},
            {'robot_description': robot_desc},]
    )

    rviz = Node(
       package='rviz2',
       executable='rviz2',
       arguments=['-d', rviz_config],
       output='screen'
    )

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{
            'config_file':  bridge_config,
        }],
        output='screen'
    )


    return LaunchDescription([set_env, gz_sim, bridge, robot_state_publisher, rviz])

