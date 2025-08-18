# Utility functions for motion processing
# Smoothing between two joint positions, create frames for specific motion

import numpy as np

from mini_pupper_behavior.math_operations import quaternion_from_euler


class MotionUtils:
    def __init__(self, joint_names):
        self.joint_names = joint_names

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
            print("Start and target poses must be lists of joint positions.")
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
            print("Start pose must be a list of joint positions.")
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
            print("Invalid side specified. Use 'left' or 'right'. Defaulting to 'left'.")
            side = 'left'

        joint_name_foot = 'lf2_lf3' if side == 'left' else 'rf2_rf3'
        joint_name_leg = 'lf1_lf2' if side == 'left' else 'rf1_rf2'
        if joint_name_foot not in joint_names:
            print(f"Joint {joint_name_foot} not found in joint names. Cannot create lift frames.")
            return []
        joint_index_foot = joint_names.index(joint_name_foot)

        if joint_name_leg not in joint_names:
            print(f"Joint {joint_name_leg} not found in joint names. Cannot create lift frames.")
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
    
    def pose_to_joint_trajectory(self, current_joint_position, positions=(0.0, 0.0, 0.0), euler_angles=(0.0, 0.0, 0.0)):
        """Convert a pose (position + orientation) to a joint trajectory.
        
        This function takes the current joint positions and modifies them based on the 
        desired pose changes while preserving the current leg configuration (sit/stand/lay).
        
        Args:
            current_joint_position (list): Current joint positions.
            positions (tuple): Target position (x, y, z) - body translation.
            euler_angles (tuple): Target orientation in Euler angles (roll, pitch, yaw).
        
        Returns:
            list: Modified joint positions for the target pose.
        """
        if not isinstance(current_joint_position, list) or len(current_joint_position) != len(self.joint_names):
            print(f"Invalid current joint position. Expected list of {len(self.joint_names)} values.")
            return current_joint_position.copy()
        
        # Start with current joint positions to preserve leg configuration
        target_joint_position = current_joint_position.copy()
        
        # Extract pose parameters
        x, y, z = positions
        roll, pitch, yaw = euler_angles
        
        # Joint mapping for easier access
        # joint_names = [
        #     'base_lf1', 'lf1_lf2', 'lf2_lf3',  # Left Front (indices 0,1,2)
        #     'base_rf1', 'rf1_rf2', 'rf2_rf3',  # Right Front (indices 3,4,5)
        #     'base_lb1', 'lb1_lb2', 'lb2_lb3',  # Left Back (indices 6,7,8)
        #     'base_rb1', 'rb1_rb2', 'rb2_rb3'   # Right Back (indices 9,10,11)
        # ]
        
        # Apply Z-axis position changes (body height)
        if z != 0.0:
            z_scale = z # Scale factor for height adjustment
            for knee_idx in [2, 5, 8, 11]:
                target_joint_position[knee_idx] += z_scale

        # Apply X-axis position changes (forward/backward lean)
        if x != 0.0:
            x_scale = x
            # Adjust front and back legs differently for forward/backward lean
            for front_hip_idx in [1, 4]:  # Front hip flexion
                target_joint_position[front_hip_idx] += x_scale
            for back_hip_idx in [7, 10]:  # Back hip flexion  
                target_joint_position[back_hip_idx] -= x_scale
                
        # Apply Y-axis position changes (sideways lean)
        if y != 0.0:
            y_scale = y
            # Adjust left and right legs differently for sideways lean
            for left_hip_idx in [0, 6]:  # Left hip abduction
                target_joint_position[left_hip_idx] += y_scale
            for right_hip_idx in [3, 9]:  # Right hip abduction
                target_joint_position[right_hip_idx] -= y_scale
        
        # Apply Roll (tilt left/right around X-axis)
        if roll != 0.0:
            roll_scale = roll   
            # Left side joints
            for left_hip_idx in [0, 6]:  # Left hip abduction
                target_joint_position[left_hip_idx] += roll_scale
            # Right side joints  
            for right_hip_idx in [3, 9]:  # Right hip abduction
                target_joint_position[right_hip_idx] -= roll_scale
                
        # Apply Pitch (tilt forward/backward around Y-axis)
        if pitch != 0.0:
            pitch_scale = pitch
            # for hip_flex_idx in [1, 4, 7, 10]:
            #     target_joint_position[hip_flex_idx] += pitch_scale
            for front_foot_idx in [2, 5]:
                target_joint_position[front_foot_idx] -= pitch_scale * 2
            for back_foot_idx in [8, 11]:
                target_joint_position[back_foot_idx] += pitch_scale

        # Apply Yaw (rotate around Z-axis)
        if yaw != 0.0:
            yaw_scale = yaw
            # Front legs: differential hip abduction for turning
            target_joint_position[0] -= yaw_scale   # Left front hip
            target_joint_position[3] += yaw_scale *0.5  # Right front hip
            # Back legs: opposite differential for coordinated turn
            # target_joint_position[6] -= yaw_scale   # Left back hip
            # target_joint_position[9] -= yaw_scale   # Right back hip
        
        return target_joint_position