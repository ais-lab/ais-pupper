#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from mini_pupper_interfaces.srv import BehaviorCommand
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose
import time
from .math_operations import quaternion_from_euler


class MiniPupperBehaviorService(Node):

    def __init__(self):
        super().__init__('mini_pupper_behavior_service')
        self.srv = self.create_service(BehaviorCommand,
                                       'behavior_command',
                                       self._behavior_callback)

        self.vel_publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.pose_publisher_ = self.create_publisher(Pose,
                                                     'reference_body_pose', 10)
        self.interval = 0.3  # seconds

    def _publish_velocity(self, linear_x=0.0, linear_y=0.0, angular_z=0.0):
        velocity_cmd = Twist()
        velocity_cmd.linear.x = linear_x
        velocity_cmd.linear.y = linear_y
        velocity_cmd.angular.z = angular_z
        self.vel_publisher_.publish(velocity_cmd)
        time.sleep(self.interval)

    def _publish_pose(self, positions=(0.0, 0.0, 0.0), euler_angles=(0.0, 0.0, 0.0)):
        pose_cmd = Pose()
        pose_cmd.position.x = positions[0]
        pose_cmd.position.y = positions[1]
        pose_cmd.position.z = positions[2]

        quaternion_x, quaternion_y, quaternion_z, quaternion_w = quaternion_from_euler(*euler_angles)
        pose_cmd.orientation.x = quaternion_x
        pose_cmd.orientation.y = quaternion_y
        pose_cmd.orientation.z = quaternion_z
        pose_cmd.orientation.w = quaternion_w
        self.pose_publisher_.publish(pose_cmd)
        time.sleep(self.interval)

    def _behavior_callback(self, request, response):
        command = request.data
        self.get_logger().info(f'Executing command: "{command}"')

        if command == 'move_forward':
            self._publish_velocity(linear_x=0.5)
        elif command == 'move_backward':
            self._publish_velocity(linear_x=-0.5)
        elif command == 'move_left':
            self._publish_velocity(linear_y=0.5)
        elif command == 'move_right':
            self._publish_velocity(linear_y=-0.5)
        elif command == 'move_up':
            self._publish_pose(positions=(0.0, 0.0, 0.5))
        elif command == 'move_down':
            self._publish_pose(positions=(0.0, 0.0, -0.5))
        elif command == 'turn_left':
            self._publish_velocity(angular_z=1.0)
        elif command == 'turn_right':
            self._publish_velocity(angular_z=-1.0)
        elif command == 'look_up':
            self._publish_pose(euler_angles=(0.0, -0.3, 0.0))
        elif command == 'look_down':
            self._publish_pose(euler_angles=(0.0, 0.3, 0.0))
        elif command == 'look_left':
            self._publish_pose(euler_angles=(0.0, 0.0, 0.3))
        elif command == 'look_right':
            self._publish_pose(euler_angles=(0.0, 0.0, -0.3))
        elif command == 'look_middle':
            self._publish_pose(euler_angles=(0.0, 0.0, 0.0))
        elif command == 'shift_left':
            self._publish_pose(euler_angles=(-0.3, 0.0, 0.0))
        elif command == 'shift_right':
            self._publish_pose(euler_angles=(0.3, 0.0, 0.0))
        elif command == 'stay':
            time.sleep(self.interval)  # Do nothing
        else:
            self.get_logger().info(f'Invalid command: "{command}"')

        # Stop the robot from moving
        self._publish_velocity()

        # Give response
        response.executed = True
        return response


def main():
    rclpy.init()
    minimal_service = MiniPupperBehaviorService()
    rclpy.spin(minimal_service)
    minimal_service.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
