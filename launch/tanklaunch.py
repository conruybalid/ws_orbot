from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='tank_control',
            executable='move_tank',
        ),

        Node(
            package='xbox_controller',
            executable='controller_node',
        ),

    ])