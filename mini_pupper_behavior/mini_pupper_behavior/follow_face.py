import rclpy
from rclpy.node import Node

class MiniPupperFollowFace(Node):
    def __init__(self):
        super().__init__('mini_pupper_follow_face')
        self.get_logger().info('Mini Pupper Follow Face Node Initialized')

          