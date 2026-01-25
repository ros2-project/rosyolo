from launch import LaunchDescription
from launch_ros.actions import Node

def  generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package='hw_pkg',
                executable='hw_node',
                name='hw_node',
                output='screen'
            )
        ]
    )