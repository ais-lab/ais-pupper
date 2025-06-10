from launch import LaunchDescription
from launch_ros.actions import Node

from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    media_file = LaunchConfiguration('media_file')
    
    return LaunchDescription([
        DeclareLaunchArgument(
            name='media_file',
            default_value='/home/ubuntu/ros2_ws/src/mini_pupper_ros/robot_state_folder/logo.png',
            description='Path to the media file to be sent'
        ),
        Node(
            package='media_publisher',
            executable='media_sender_node',
            name='media_sender',
            output='screen',
            arguments=[media_file],
        )
    ])
