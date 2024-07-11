from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='get_pic',
            executable='video_publisher',
        ),
        Node(
            package='get_pic',
            executable='video_subscriber',
        ),

        Node(
            package='image_processing',
            executable='rgb_image_service',
        ),

        Node(
            package='get_pic',
            executable='zed_publisher',
        ),

        Node(
            package='image_processing',
            executable='zed_location_service',
        ),
    ])