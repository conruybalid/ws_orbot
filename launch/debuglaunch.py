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
            executable='arm_location_service',
        ),
        # Node(
        #     package='get_pic',
        #     executable='masked_image_subscriber',
        # ),

        # Node(
        #     package='pick_apple',
        #     executable='search_apple',
        # ),

        # Node(
        #     package='pick_apple',
        #     executable='pick_apple_action_server',
        # ),


        Node(
            package='arm_control',
            executable='arm_move_action',
        ),
    ])