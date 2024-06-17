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
            package='get_pic',
            executable='masked_image_subscriber',
        ),

        # Node(
        #     package='arm_control',
        #     executable='arm_control',
        # ),
    ])