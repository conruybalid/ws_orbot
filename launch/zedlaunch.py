from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='get_pic',
            executable='video_subscriber',
        ),

        Node(
            package='get_pic',
            executable='zed_publisher',
        ),

        Node(
            package='image_processing',
            executable='zed_location_service',
            arguments=['--ros-args', '--log-level', 'INFO']
        ),
    ])