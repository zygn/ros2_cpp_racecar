import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    ld = LaunchDescription()

    config = os.path.join(
        get_package_share_directory('ros2_cpp_racecar'),
        'config',
        'params.yaml'
        )
    
    ld.add_action(
        Node(
            package='ros2_cpp_racecar',
            namespace='ros2_cpp_racecar',
            executable='racecar_cpp',
            name='racecar',
            output='screen',
            parameters=[config]
        )
    )

    return ld