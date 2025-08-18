from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    behavior_server_node = Node(
            package="mini_pupper_behavior",
            namespace="",
            executable="behavior_server",
            name="behavior_server",
        )
    behavior_client_node = Node(
            package="mini_pupper_behavior",
            namespace="",
            executable="behavior_client",
            name="behavior_client",
        )
    pose_controller_node = Node(
            package="mini_pupper_behavior",
            namespace="",
            executable="pose_controller",
            name="pose_controller",
        )
    detect_face_node = Node(
            package="mini_pupper_behavior",
            namespace="",
            executable="detect_face",
            name="detect_face",
        )
    return LaunchDescription([
        behavior_server_node,
        behavior_client_node,
        pose_controller_node,
        # detect_face_node
    ])
