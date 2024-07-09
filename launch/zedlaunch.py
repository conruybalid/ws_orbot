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
        #     package='get_pic',
        #     executable='zed_distance_pub',
        # ),

        Node(
            package='image_processing',
            executable='zed_location_service',
        ),
    ])