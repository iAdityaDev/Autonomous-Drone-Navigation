import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node

def generate_launch_description():	
    # get the required paths of packages & files
    package_path = get_package_share_directory('drone_delivery')
    bridge_config = os.path.join(package_path, 'config', 'bridge.yaml')
    
    
    ros_gz_bridge = Node(package="ros_gz_bridge", 
                executable="parameter_bridge",
                parameters = [
                    {'config_file': bridge_config}],
                )
    
    mavros = ExecuteProcess(
        cmd=[
            'ros2', 'launch', 'mavros', 'apm.launch.py',
            'fcu_url:=udp://:14540@127.0.0.1:14557'
        ],
        output='screen'
    )
    
    
    return LaunchDescription([
    Node(
            package='drone_delivery',
            executable='AlignDrone',
            name='AlignDrone',
            output='screen'
        ),
    
    Node(
            package='drone_delivery',
            executable='controller',
            name='controller',
            output='screen'
        ),
        mavros,
        ros_gz_bridge
        
    ])
