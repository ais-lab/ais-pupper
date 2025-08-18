#!/usr/bin/env python3

import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from mini_pupper_interfaces.action import BehaviorCommand
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_msgs.msg import Header
from rclpy.duration import Duration
from builtin_interfaces.msg import Time
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose
import time
from .math_operations import quaternion_from_euler
from .motion_loader import MotionLoader
from .motion_utils import MotionUtils


class MiniPupperBehaviorActionServer(Node):

    def __init__(self):
        super().__init__('mini_pupper_behavior_action_server')
        self.action_server = ActionServer(self, BehaviorCommand,
                                       'behavior_command',
                                       execute_callback=self._behavior_callback,
                                       cancel_callback=self._cancel_callback)

        self.vel_publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.pose_publisher_ = self.create_publisher(Pose,
                                                     'reference_body_pose', 10)
        self.joint_pose_target_publisher = self.create_publisher(JointTrajectory, 'joint_pose_targets', 10)
        self.joint_trajectory_subscriber = self.create_subscription(JointTrajectory, 'joint_group_effort_controller/joint_trajectory', self._joint_trajectory_callback, 10)
        self.interval = 0.3  # seconds

        self.joint_names = [
            'base_lf1', 'lf1_lf2', 'lf2_lf3',
            'base_rf1', 'rf1_rf2', 'rf2_rf3',
            'base_lb1', 'lb1_lb2', 'lb2_lb3',
            'base_rb1', 'rb1_rb2', 'rb2_rb3'
        ]

        # Initialize motion management classes
        self.motion_loader = MotionLoader(self.joint_names)
        self.motion_utils = MotionUtils(self.joint_names)
        
        # Current state tracking
        self.current_pose = None
        self.current_state = None
        
        # Behavior execution control
        self.executing_sequence = False
        self.current_goal_handle = None

    def _joint_trajectory_callback(self, msg):
        """Handle incoming joint trajectory messages."""
        # Convert array('d', ...) to a standard Python list
        self.current_pose = list(msg.points[0].positions)
        if self.current_state is None:
            self.current_state = self.motion_loader.recover_current_state(self.current_pose)
            self.get_logger().info(f'Initial state detected: {self.current_state}')

    def _joints_are_equal(self, joint1List, joint2List, tolerance=1e-5):
        """Check if two joints are equal within a tolerance."""
        if not isinstance(joint1List, list) or not isinstance(joint2List, list):
            self.get_logger().error(f"Expected lists, got {type(joint1List)} and {type(joint2List)}")
            return False
        if len(joint1List) != len(joint2List):
            return False
        return all(abs(j1 - j2) < tolerance for j1, j2 in zip(joint1List, joint2List))

    def _publish_velocity(self, linear_x=0.0, linear_y=0.0, angular_z=0.0):
        velocity_cmd = Twist()
        velocity_cmd.linear.x = linear_x
        velocity_cmd.linear.y = linear_y
        velocity_cmd.angular.z = angular_z
        self.vel_publisher_.publish(velocity_cmd)
        time.sleep(self.interval)

    def pose_to_joint_trajectory(self, current_joint_position, positions=(0.0, 0.0, 0.0), euler_angles=(0.0, 0.0, 0.0)):
        """Convert a pose (position + orientation) to a joint trajectory."""
        joint_trajectory = self.motion_utils.pose_to_joint_trajectory(current_joint_position, positions, euler_angles)
        return joint_trajectory

    def _publish_pose(self, positions=(0.0, 0.0, 0.0), euler_angles=(0.0, 0.0, 0.0)):
        """
        Publish pose transition using joint positions instead of pose commands.
        This preserves the current leg configuration (sit/stand/lay) while applying pose changes.
        """
        # pose_cmd = Pose()
        # pose_cmd.position.x = positions[0]
        # pose_cmd.position.y = positions[1]
        # pose_cmd.position.z = positions[2]

        # quaternion_x, quaternion_y, quaternion_z, quaternion_w = quaternion_from_euler(*euler_angles)
        # pose_cmd.orientation.x = quaternion_x
        # pose_cmd.orientation.y = quaternion_y
        # pose_cmd.orientation.z = quaternion_z
        # pose_cmd.orientation.w = quaternion_w
        # self.pose_publisher_.publish(pose_cmd)
        # time.sleep(self.interval)
        # Check if we have current pose available
        if self.current_pose is None:
            self.get_logger().warning('No current pose available, cannot apply pose changes')
            return

        # Convert pose parameters to target joint positions
        target_joint_positions = self.pose_to_joint_trajectory(
            self.current_pose, positions, euler_angles
        )
        
        # Create smooth transition from current to target pose
        transition_frames = self.motion_utils.smoothing_between_poses(
            self.current_pose, target_joint_positions, steps=400  # ~0.3s at 200Hz
        )
        
        # Execute the transition
        for frame in transition_frames:
            self._publish_joint_trajectory(frame)
            time.sleep(0.005)  # 200Hz timing
        
        self.get_logger().info(f'Completed pose transition: pos={positions}, euler={euler_angles}')

    def _publish_joint_trajectory(self, joint_positions):
        """Publish a single joint trajectory point."""
        # Ensure positions are native Python floats for ROS2 message compatibility
        if isinstance(joint_positions, (list, tuple)):
            joint_positions = [float(pos) for pos in joint_positions]
        else:
            self.get_logger().error(f"Expected list of joint positions, got {type(joint_positions)}: {joint_positions}")
            return
            
        now = self.get_clock().now().to_msg()
        joint_trajectory_msg = JointTrajectory()
        joint_trajectory_msg.header = Header()
        joint_trajectory_msg.header.stamp = Time(sec=now.sec, nanosec=now.nanosec)
        joint_trajectory_msg.joint_names = self.joint_names
        point = JointTrajectoryPoint()
        point.positions = joint_positions
        joint_trajectory_msg.points.append(point)
        joint_trajectory_msg.points[0].time_from_start = Duration(seconds=0.02).to_msg()  # Match deluxe script
        self.joint_pose_target_publisher.publish(joint_trajectory_msg)

    def _execute_joint_sequence(self, frames, goal_handle, sequence_name="sequence"):
        """Execute a sequence of joint positions."""
        self.executing_sequence = True
        self.current_goal_handle = goal_handle
        
        total_frames = len(frames)
        self.get_logger().info(f'Executing sequence "{sequence_name}" with {total_frames} frames')
        
        if total_frames == 0:
            self.get_logger().warning(f'No frames to execute for sequence "{sequence_name}"')
            self.executing_sequence = False
            return None
        
        for frame_idx, joint_positions in enumerate(frames):
            # Check if goal was cancelled
            if goal_handle.is_cancel_requested:
                self.get_logger().info('Goal cancelled during sequence execution')
                goal_handle.canceled()
                self.executing_sequence = False
                return BehaviorCommand.Result(executed=False)
            
            # Publish joint positions
            self._publish_joint_trajectory(joint_positions)
            
            # Provide feedback every 40 frames or on first/last frame (reduces overhead at 200Hz)
            if frame_idx % 40 == 0 or frame_idx == total_frames - 1:
                feedback_msg = BehaviorCommand.Feedback()
                feedback_msg.status = f'{sequence_name}: frame {frame_idx+1}/{total_frames}'
                goal_handle.publish_feedback(feedback_msg)
            
            # Sleep for frame timing (200Hz = 0.005s per frame like deluxe script)
            time.sleep(0.005)
        
        self.get_logger().info(f'Completed sequence "{sequence_name}"')
        self.executing_sequence = False
        self.current_goal_handle = None
        return None  # Continue execution

    def _execute_behavior_sequence(self, behavior, goal_handle):
        """Execute a complete behavior sequence."""
        # Ensure we have current pose and state
        if self.current_pose is None:
            self.get_logger().error('No current pose available for behavior execution')
            goal_handle.abort()
            return BehaviorCommand.Result(executed=False)
            
        if self.current_state is None:
            self.get_logger().warning('No current state detected, attempting to recover from current pose')
            self.current_state = self.motion_loader.recover_current_state(self.current_pose)
            self.get_logger().info(f'Recovered state: {self.current_state}')
        
        moves_frames = {}
        current_pose = self.current_pose.copy()
        current_state = self.current_state
        target_pose = None

        self.get_logger().info(f'Executing behavior sequence: {current_state} -> {behavior}')
            
        if behavior in self.motion_loader.states:
            # Check if already in target state
            if behavior == current_state:
                self.get_logger().info(f'Already in state: {behavior}')
                goal_handle.succeed()
                return BehaviorCommand.Result(executed=True)
                
            # State transition
            try:
                self.get_logger().info(f"Loading transition: {current_state} -> {behavior}")
                transition_frames = self.motion_loader.load_transition_state(current_state, behavior)
                
                if transition_frames:
                    self.get_logger().info(f"Loaded {len(transition_frames)} transition frames")

                    # Get closest transition frame index
                    closest_idx = self.motion_loader.get_closest_transition_frame_idx(current_pose, current_state, behavior)
                    
                    # Add smoothing
                    smoothing_key = f"smoothing_{current_state}_{behavior}"
                    smoothing_frames = self.motion_utils.smoothing_between_poses(
                        current_pose, transition_frames[closest_idx], steps=200  # Match deluxe script
                    )
                    moves_frames[smoothing_key] = smoothing_frames
                    self.get_logger().info(f"Added {len(smoothing_frames)} smoothing frames")
                    
                    # Add transition
                    transition_key = f"transition_{current_state}_{behavior}"
                    transition_main_frames = transition_frames[closest_idx:] # Use frames from closest index to end
                    moves_frames[transition_key] = transition_main_frames
                    self.get_logger().info(f"Added {len(transition_main_frames)} main transition frames")
                    self.target_pose = transition_frames[-1]  # Last frame as target pose
                    target_pose = transition_frames[-1]  # Last frame as target pose
                else:
                    self.get_logger().warning(f"No transition found: {current_state} -> {behavior}")
            except Exception as e:
                self.get_logger().error(f"Error loading transition {current_state} -> {behavior}: {e}")
                
        # Execute all sequences
        try:
            for key, frames in moves_frames.items():
                self.get_logger().info(f"Starting execution of: {key}")
                result = self._execute_joint_sequence(frames, goal_handle, key)
                if result is not None:  # Cancelled
                    return result
        except Exception as e:
            self.get_logger().error(f"Error executing behavior sequence: {e}")
            goal_handle.abort()
            return BehaviorCommand.Result(executed=False)

        # wait until self.current pose matches target_pose at 10^-4, without blocking
        while not self._joints_are_equal(self.current_pose, target_pose, tolerance=1e-4):
            rclpy.spin_once(self)

        # Update internal state
        self.current_state = behavior
        self.get_logger().info(f"Behavior sequence completed. New state: {self.current_state}")
        
        # Explicitly succeed the goal
        goal_handle.succeed()
        return BehaviorCommand.Result(executed=True)

    def _behavior_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')
        
        request = goal_handle.request
        command = request.data
        self.get_logger().info(f'Executing command: "{command}"')

        # Provide initial feedback
        feedback_msg = BehaviorCommand.Feedback()
        feedback_msg.status = f'Starting {command}'
        goal_handle.publish_feedback(feedback_msg)

        # Handle complex behavior sequences
        if command == 'default':
            smoothing_frames = self.motion_utils.smoothing_between_poses(
                self.current_pose, self.motion_loader.get_default_joint_poses(), steps=200
            )
            for frame in smoothing_frames:
                self._publish_joint_trajectory(frame)
                time.sleep(0.005)
            goal_handle.succeed()
            return BehaviorCommand.Result(executed=True)
        elif command in ['yes', 'no', 'shake', 'bow']:
            return self._execute_predefined_sequence(command, goal_handle)
        elif command in self.motion_loader.states:
            # Single state transition
            return self._execute_state_transition(command, goal_handle)
        elif command == 'breathing':
            return self._execute_breathing(goal_handle)
        elif command == 'pause' or command == 'stay':
            # Handle pause command
            self.get_logger().info('Pausing for 3 seconds...')
            time.sleep(3)
            goal_handle.succeed()
            return BehaviorCommand.Result(executed=True)
        # Handle simple pose/velocity commands
        elif command == 'move_forward':
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
        elif command == 'shift_left':
            self._publish_pose(euler_angles=(-0.3, 0.0, 0.0))
        elif command == 'shift_right':
            self._publish_pose(euler_angles=(0.3, 0.0, 0.0))
        elif command == 'stay' or command == 'pause'    :
            time.sleep(3)  # Do nothing
        else:
            self.get_logger().warning(f'Unknown command: "{command}"')

        # Stop the robot from moving (for simple commands)
        # self._publish_velocity()

        # Set result
        goal_handle.succeed()
        result = BehaviorCommand.Result()
        result.executed = True
        return result

    def _execute_predefined_sequence(self, sequence_name, goal_handle):
        """Execute predefined sequences like yes, no, shake, bow."""
        if sequence_name == 'yes':
            sequence = self.motion_loader.yes
        elif sequence_name == 'no':
            sequence = self.motion_loader.no
        elif sequence_name == 'shake':
            sequence = self.motion_loader.shake
        elif sequence_name == 'bow':
            sequence = self.motion_loader.bow
        else:
            goal_handle.abort()
            return BehaviorCommand.Result(executed=False)
        
        # Execute each command in the sequence
        for command in sequence:
            feedback_msg = BehaviorCommand.Feedback()
            feedback_msg.status = f'{sequence_name}: executing {command}'
            goal_handle.publish_feedback(feedback_msg)
            
            if command == 'look_up':
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
            elif command == 'pause':
                time.sleep(2)  # Do nothing
                
        goal_handle.succeed()
        return BehaviorCommand.Result(executed=True)

    def _execute_state_transition(self, target_state, goal_handle):
        """Execute a single state transition."""
        # if target_state == self.current_state:
        #     self.get_logger().info(f'Already in state: {target_state}')
        #     goal_handle.succeed()
        #     return BehaviorCommand.Result(executed=True)
        
        return self._execute_behavior_sequence(target_state, goal_handle)

    def _execute_breathing(self, goal_handle):
        """Execute breathing motion."""
        breathing_frames = self.motion_utils.create_idle_respiration_frames(
            self.joint_names, self.current_pose, total_breath_frames=600
        )
        result = self._execute_joint_sequence(breathing_frames, goal_handle, "breathing")
        if result is not None:
            return result
        
        goal_handle.succeed()
        return BehaviorCommand.Result(executed=True)

    
    def _cancel_callback(self, goal_handle):
        self.get_logger().info('Canceling goal...')
        goal_handle.canceled()
        feedback_msg = BehaviorCommand.Feedback()
        feedback_msg.status = 'Goal canceled'
        goal_handle.publish_feedback(feedback_msg)
        return BehaviorCommand.Result(executed=False)

def main():
    rclpy.init()
    minipupper_action_server = MiniPupperBehaviorActionServer()
    rclpy.spin(minipupper_action_server)

if __name__ == '__main__':
    main()
