#!/usr/bin/env python3

import numpy as np
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_msgs.msg import Header
from builtin_interfaces.msg import Time
from rclpy.duration import Duration

class JointPublisher(Node):
    def __init__(self):
        super().__init__('manual_joint_publisher')
        self.joint_pose_target_publisher = self.create_publisher(JointTrajectory, '/joint_pose_targets', 10)

        self.joint_names = [
            'base_lf1', 'lf1_lf2', 'lf2_lf3',
            'base_rf1', 'rf1_rf2', 'rf2_rf3',
            'base_lb1', 'lb1_lb2', 'lb2_lb3',
            'base_rb1', 'rb1_rb2', 'rb2_rb3'
        ]

        self.default_joint_poses = [
            -0.080311, 1.078067, -1.983427, # FL
            0.080311, 1.078067, -1.983427, # FR
            -0.080311, 1.078067, -1.983427, # BL
            0.080311, 1.078067, -1.983427  # BR
        ]

        self.current_pose = self.default_joint_poses.copy()
        self.current_state = "stand"
        states= ["sit", "stand", "lay"]
        events = ["shake", "idle"]
        self.moves_frames = {}
        pause_duration_frames = 600  # Number of frames to pause at each state

        self.behavior_sequence = [
            "sit_0",
            # "pause_0"
            "breathing_0",
            "stand_0",
            # "pause_1",
            "breathing_1",
            # "shake_0",
            # "sit_1",
            # "breathing_2",
            "lay_0",
            # "pause_2",
            # "breathing_3",
            "sit_1",
            "pause_3",
            # "breathing_3",
            # "lay_1",
            # "breathing_4",
        ]
        retargeted_motion_path = '/home/minipc/aislab_ws/src/ais_pupper/retargeted_motions/'

        self.moves_frames[self.current_state] = [self.default_joint_poses.copy()]
        for behavior in self.behavior_sequence:
            behavior_name = behavior.split('_')[0] if '_' in behavior else behavior
            behavior_index = behavior.split('_')[1] if '_' in behavior else '0'
            if behavior_name in states and behavior_name != self.current_state:
                try:
                    self.get_logger().info(f"Loading joint positions for {behavior_name} state.")
                    transition_state = self.load_joint_positions(
                        file_path=f"{retargeted_motion_path}{self.current_state}_{behavior_name}_12_joint_pos_200fps.txt"
                    )
                except FileNotFoundError:
                    self.get_logger().error(f"File for transition from {self.current_state} to {behavior_name} not found.")
                else:
                    self.moves_frames[f"smoothing_{self.current_state}_{behavior_name}_{behavior_index}"] = self.smoothing_between_poses(
                        start_pose=self.current_pose,
                        target_pose=transition_state[0]
                    )
                    self.moves_frames[f"transition_{self.current_state}_{behavior_name}_{behavior_index}"] = transition_state[1:]
                    self.current_pose = self.moves_frames[f"transition_{self.current_state}_{behavior_name}_{behavior_index}"][-1]
                    self.current_state = behavior_name
            elif behavior_name in events:
                try:
                    self.get_logger().info(f"Loading joint positions for {behavior_name} event.")
                    event_frames = self.load_joint_positions(
                        file_path=f"{retargeted_motion_path}{behavior_name}_12_joint_pos_200fps.txt"
                    )
                except FileNotFoundError:
                    self.get_logger().error(f"File for event {behavior_name} not found.")
                else:
                    self.moves_frames[f"smoothing_{self.current_state}_{behavior_name}_{behavior_index}"] = self.smoothing_between_poses(
                        start_pose=self.current_pose,
                        target_pose=event_frames[0]
                    )
                    self.moves_frames[f"event_{self.current_state}_{behavior_name}_{behavior_index}"] = event_frames[1:]
                    self.current_pose = self.moves_frames[f"event_{self.current_state}_{behavior_name}_{behavior_index}"][-1]
            elif behavior_name == "pause":
                self.get_logger().info(f"Pausing at current state: {self.current_state} (index: {behavior_index}).")
                # Add a pause frame with index to avoid overwriting
                self.moves_frames[f"pause_{self.current_state}_{behavior_index}"] = [self.current_pose.copy()] * pause_duration_frames

            elif behavior_name == "breathing":
                self.get_logger().info(f"Creating idle respiration frames for {self.current_state} state.")
                breathing_frames = self.create_idle_respiration_frames(
                    joint_names=self.joint_names,
                    start_pose=self.current_pose,
                    total_breath_frames=pause_duration_frames,
                    breaths_per_minute=50.0,
                    frame_rate=200,
                    breath_height=0.30,
                    pause_ratio=0.02,
                    oval_radius=0.02
                )
                self.moves_frames[f"breathing_{self.current_state}_{behavior_index}"] = breathing_frames
                self.current_pose = breathing_frames[-1]
            else:
                self.get_logger().warning(f"Unknown behavior: {behavior_name}. Skipping.")
                continue

        #  if last state is not 'stand', add transition to 'stand'
        if self.current_state != "stand":
            try:
                self.get_logger().info(f"End of sequence, transitioning to stand from {self.current_state}.")
                transition_to_stand = self.load_joint_positions(
                    file_path=f"{retargeted_motion_path}{self.current_state}_stand_12_joint_pos_200fps.txt"
                )
            except FileNotFoundError:
                self.get_logger().error(f"File for transition from {self.current_state} to stand not found.")
            else:
                self.moves_frames[f"final_smoothing_{self.current_state}_stand"] = self.smoothing_between_poses(
                    start_pose=self.current_pose,
                    target_pose=transition_to_stand[0]
                )
                self.moves_frames[f"final_transition_{self.current_state}_stand"] = transition_to_stand[1:]
                self.current_pose = self.moves_frames[f"final_transition_{self.current_state}_stand"][-1]
                self.current_state = "stand"

        #  smoothly transition to the default pose
        if self.current_state == "stand":
            self.get_logger().info("Transitioning to default joint poses.")
            self.moves_frames[f"final_smoothing_{self.current_state}_default"] = self.smoothing_between_poses(
                start_pose=self.current_pose,
                target_pose=self.default_joint_poses
            )
        # Prepare ordered list of all keys and frames
        self.sequence_keys = list(self.moves_frames.keys())
        self.sequence_frames = [self.moves_frames[k] for k in self.sequence_keys]
        self.sequence_lengths = [len(frames) for frames in self.sequence_frames]
        self.total_frames = sum(self.sequence_lengths)
        self.current_sequence_idx = 0
        self.current_frame_idx = 0

        timer_period = 0.005  # 200Hz
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def smoothing_between_poses(self, start_pose, target_pose, steps=400):
        """
        Transition between two poses in a smooth manner.
        
        Args:
            start_pose (list): Initial joint positions.
            target_pose (list): Target joint positions.
            steps (int): Number of steps for the transition.
            
        Returns:
            list: List of joint positions over time (frames).
        """
        if not isinstance(start_pose, list) or not isinstance(target_pose, list):
            self.get_logger().error("Start and target poses must be lists of joint positions.")
            return []

        frames = []
        for step in range(steps):
            new_position = [
                start + (target - start) * step / (steps - 1)
                for start, target in zip(start_pose, target_pose)
            ]
            frames.append(new_position)
        return frames

    def create_idle_respiration_frames(self, joint_names, start_pose, total_breath_frames=600, breaths_per_minute=25.0, frame_rate=200, breath_height=0.30, pause_ratio=0.02, oval_radius=0.03):
        """
        Create breathing motion frames using a sinusoidal profile.
        
        Args:
            joint_names (list): List of joint names.
            start_pose (list): Initial joint positions.
            breaths_per_minute (float): Desired breathing rate.
            frame_rate (int): Number of frames per second.
            breath_height(float): Max amplitude of joint motion in degrees.
            pause_ratio (float): Ratio of pause duration at the peak of the inhale.
            
        Returns:
            list: List of joint positions over time (frames).
        """
        if not isinstance(start_pose, list):
            self.get_logger().error("Start pose must be a list of joint positions.")
            return []
        
        # ---- Breathing duration in frames ----
        seconds_per_breath = 60.0 / breaths_per_minute
        # breath_sec_to_frames = int(seconds_per_breath * frame_rate)#ex: 2 seconds per breath -> 400 frames at 200fps


        # Divide into inhale and exhale
        breath_duration_steps = total_breath_frames // 2
        pause_duration_steps = int(total_breath_frames * pause_ratio)

        frames = []
        # Joint indices
        lf_leg_index = joint_names.index('lf1_lf2')
        rf_leg_index = joint_names.index('rf1_rf2')
        lf_foot_index = joint_names.index('lf2_lf3')
        rf_foot_index = joint_names.index('rf2_rf3')
        lb_leg_index = joint_names.index('lb1_lb2')
        rb_leg_index = joint_names.index('rb1_rb2')

        # Inhale phase
        for step in range(1, breath_duration_steps + 1):
            theta = np.pi * step / (breath_duration_steps - 1)  # From 0 to π
            forward_offset = oval_radius * np.sin(theta)
            upward_phase = breath_height * (1 - np.cos(theta)) / 2

            new_position = start_pose.copy()
            new_position[lf_leg_index] += forward_offset
            new_position[rf_leg_index] += forward_offset
            new_position[lb_leg_index] += forward_offset
            new_position[rb_leg_index] += forward_offset

            new_position[lf_foot_index] += upward_phase
            new_position[rf_foot_index] += upward_phase

            frames.append(new_position)

        # Pause at the peak of inhale
        hold_position = frames[-1].copy()
        for step in range(pause_duration_steps):
            frames.append(hold_position)

        # Create frames for returning to the original position
        for step in range(1, breath_duration_steps + 1):
            theta = np.pi * (1 - step / (breath_duration_steps - 1))  # From π to 0
            forward_offset = oval_radius * np.sin(theta)
            upward_phase = breath_height * (1 - np.cos(theta)) / 2

            new_position = start_pose.copy()
            new_position[lf_leg_index] += forward_offset 
            new_position[rf_leg_index] += forward_offset
            new_position[lb_leg_index] += forward_offset
            new_position[rb_leg_index] += forward_offset

            new_position[lf_foot_index] += upward_phase
            new_position[rf_foot_index] += upward_phase

            frames.append(new_position)

        return frames

    def create_foreleg_lift_frames(self, joint_names, start_pose, side='left', height=0.01, steps=10):
        """ Create frames for lifting a foreleg joint from a specified height and then lowering it back down.
        Args:
            start_pose (dict): The starting joint positions.
            side (str): 'left' or 'right' to specify which foreleg to lift.
            height (float): The height to lift the joint.
            steps (int): The number of steps to create for the lift and lower motion.
        Returns:
            list: A list of joint position dictionaries representing the frames for the lift and lower motion.
        """
        if side not in ['left', 'right']:
            self.get_logger().error("Invalid side specified. Use 'left' or 'right'. Defaulting to 'left'.")
            side = 'left'

        joint_name_foot = 'lf2_lf3' if side == 'left' else 'rf2_rf3'
        joint_name_leg = 'lf1_lf2' if side == 'left' else 'rf1_rf2'
        if joint_name_foot not in joint_names:
            self.get_logger().error(f"Joint {joint_name_foot} not found in joint names. Cannot create lift frames.")
            return []
        joint_index_foot = joint_names.index(joint_name_foot)

        if joint_name_leg not in joint_names:
            self.get_logger().error(f"Joint {joint_name_leg} not found in joint names. Cannot create lift frames.")
            return []
        joint_index_leg = joint_names.index(joint_name_leg)

        frames = []
        # Create frames for lifting the joint
        for step in range(steps):
            # Calculate the new position for the joint
            new_position = start_pose.copy()
            new_position[joint_index_foot] -= height * (step / (steps - 1))
            # print the added height for debugging
            # self.get_logger().info(f"Step {step}: Adding height {height * (step / (steps - 1))} to joint {joint_name_foot} at index {joint_index_foot}")
            new_position[joint_index_leg] -= height * (step / (steps - 1))
            # positions are in a list, names are stripped
            new_position = [new_position[joint_idx] for joint_idx in range(len(joint_names))]
            frames.append(new_position)

        # Create frames for holding the joint at the lifted position for the number of steps
        hold_position = frames[-1].copy()
        for step in range(100):
            frames.append(hold_position)

        # Create frames for lowering the joint from the last lifted position back to the original position
        for step in range(steps):
            new_position = hold_position.copy()
            new_position[joint_index_foot] += height * (step / (steps - 1))
            # print the added height for debugging
            # self.get_logger().info(f"Step {step}: Lowering height {height * (step / (steps - 1))} from joint {joint_name_foot} at index {joint_index_foot}")
            new_position[joint_index_leg] += height * (step / (steps - 1))
            new_position = [new_position[joint_idx] for joint_idx in range(len(joint_names))]
            frames.append(new_position)

        return frames

    def load_joint_positions(self, file_path=None):
        try:
            self.get_logger().info(f"Loading joint positions from: {file_path}")
            with open(file_path, 'r') as file:
                lines = file.readlines()
                all_joint_positions = []
                # Parse header to get joint order in file
                header = lines[0].strip().split(', ')
                # print(f"Header joints: {header}")
                # Build index mapping from header to self.joint_names
                index_map = [header.index(joint) for joint in self.joint_names]
                for idx, line in enumerate(lines):
                    if idx == 0:
                        continue  # skip header
                    line = line.strip()
                    positions = [float(pos) for pos in line.split(',')]
                    # Reorder positions according to self.joint_names
                    reordered = [positions[i] for i in index_map]
                    all_joint_positions.append(reordered)
                return all_joint_positions
        except FileNotFoundError:
            self.get_logger().error(f"File not found: {file_path}")
            return []

    def timer_callback(self):
        # Get current state and frame
        key = self.sequence_keys[self.current_sequence_idx]
        frames = self.sequence_frames[self.current_sequence_idx]
        joint_positions = frames[self.current_frame_idx]
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
        joint_trajectory_msg.points[0].time_from_start = Duration(seconds=0.02).to_msg()
        self.joint_pose_target_publisher.publish(joint_trajectory_msg)

        self.get_logger().info(f'[{key}] Frame {self.current_frame_idx+1}/{len(frames)}')

        # Advance frame
        self.current_frame_idx += 1
        if self.current_frame_idx >= len(frames):
            self.current_sequence_idx += 1
            self.current_frame_idx = 0
            if self.current_sequence_idx >= len(self.sequence_keys):
                self.get_logger().info("Sequence finished.")
                self.timer.cancel()

def main(args=None):
    rclpy.init(args=args)
    joint_publisher = JointPublisher()
    rclpy.spin(joint_publisher)
    joint_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
