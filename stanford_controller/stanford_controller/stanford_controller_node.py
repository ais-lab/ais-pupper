import numpy as np
import rclpy
from champ_msgs.msg import ContactsStamped
from mini_pupper_interfaces.msg import Command
from rclpy.node import Node
from sensor_msgs.msg import Imu, JointState
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from transforms3d.euler import euler2mat, quat2euler

from .Config import Configuration
from .gait_controller import GaitController
from .Kinematics import four_legs_inverse_kinematics
from .stance_controller import StanceController
from .State import BehaviorState, State
from .swing_controller import SwingController
from .Utilities import clipped_first_order_filter, convert_to_JTP_positions


class StanfordControllerNode(Node):
    """ROS 2 Node for StanfordController."""

    def __init__(self, config, inverse_kinematics):
        super().__init__('stanford_controller_node')

        # Read parameters
        self.declare_parameter('orientation_from_imu', False)
        self.orientation_from_imu = self.get_parameter(
            'orientation_from_imu').get_parameter_value().bool_value
        self.get_logger().info(f"use_imu: {self.orientation_from_imu}")

        self.declare_parameter('publish_joint_control', False)
        self.publish_joint_control = self.get_parameter(
            'publish_joint_control').get_parameter_value().bool_value

        self.declare_parameter('publish_states', False)
        self.publish_states = self.get_parameter(
            'publish_states').get_parameter_value().bool_value
        
        self.declare_parameter('publish_foot_contacts', False)
        self.publish_foot_contacts = self.get_parameter(
            'publish_foot_contacts').get_parameter_value().bool_value
        
        self.declare_parameter('publish_joint_states', False)
        self.publish_joint_states = self.get_parameter(
            'publish_joint_states').get_parameter_value().bool_value

        # Configuration and initialization
        self.joint_names = [
            "base_lf1", "lf1_lf2", "lf2_lf3",
            "base_rf1", "rf1_rf2", "rf2_rf3",
            "base_lb1", "lb1_lb2", "lb2_lb3",
            "base_rb1", "rb1_rb2", "rb2_rb3"
        ]
        self.config = config
        self.inverse_kinematics = inverse_kinematics
        self.smoothed_yaw = 0.0
        self.dance_active_state = False

        self.contact_modes = np.zeros(4)
        self.gait_controller = GaitController(self.config)
        self.swing_controller = SwingController(self.config)
        self.stance_controller = StanceController(self.config)

        self.hop_transition_mapping = {
            BehaviorState.REST: BehaviorState.HOP,
            BehaviorState.HOP: BehaviorState.FINISHHOP,
            BehaviorState.FINISHHOP: BehaviorState.REST,
            BehaviorState.TROT: BehaviorState.HOP,
        }
        self.trot_transition_mapping = {
            BehaviorState.REST: BehaviorState.TROT,
            BehaviorState.TROT: BehaviorState.REST,
            BehaviorState.HOP: BehaviorState.TROT,
            BehaviorState.FINISHHOP: BehaviorState.TROT,
        }
        self.activate_transition_mapping = {
            BehaviorState.DEACTIVATED: BehaviorState.REST,
            BehaviorState.REST: BehaviorState.DEACTIVATED,
        }

        # ROS 2 publishers and subscribers
        self.current_command = None
        self.command_subscriber = self.create_subscription(
            Command,
            'robot_command',
            self.command_callback,
            10
        )

        if self.orientation_from_imu:
            self.imu_subscriber = self.create_subscription(
                Imu,
                'imu/data',
                self.imu_callback,
                10
            )

        self.joint_trajectory_publisher = self.create_publisher(
            JointTrajectory,
            'joint_group_effort_controller/joint_trajectory',
            10
        )
        self.state_publisher = self.create_publisher(String, 'state_log', 10)

        self.foot_contact_publisher = self.create_publisher(
            ContactsStamped,
            'foot_contacts',
            10
        )
        
        self.joint_states_publisher = self.create_publisher(
            JointState,
            'joint_states',
            10
        )


        self.state = State()
        self.quat_orientation = np.array([1, 0, 0, 0])
        # self.timer = self.create_timer(self.config.dt, self.control_loop)

    def imu_callback(self, msg):
        self.quat_orientation = np.array([
            msg.orientation.w,
            msg.orientation.x,
            msg.orientation.y,
            msg.orientation.z
        ])

    def publish_foot_contact(self):
        if not self.publish_foot_contacts:
            return
            
        contacts_msg = ContactsStamped()
        contacts_msg.header.stamp = self.get_clock().now().to_msg()
        contacts_msg.header.frame_id = 'base_link'
        
        # Get contact states from gait controller
        contact_states = self.gait_controller.contacts(self.state.ticks)
        contacts_msg.contacts = [bool(contact) for contact in contact_states]
        
        self.foot_contact_publisher.publish(contacts_msg)

    def publish_joint_state(self):
        """Publish joint states similar to CHAMP controller."""
        if not self.publish_joint_states:
            return
            
        # Create JointState message (same as CHAMP)
        joints_msg = JointState()
        joints_msg.header.stamp = self.get_clock().now().to_msg()
        joints_msg.header.frame_id = 'base_link'
        
        # Set joint names (already defined in your code)
        joints_msg.name = self.joint_names
        
        # Convert 3x4 joint angles to 12x1 array (reuse your existing utility)
        joint_positions = convert_to_JTP_positions(self.state.joint_angles)
        joints_msg.position = joint_positions
        
        # Optional: Add velocities and efforts (zeros for now)
        joints_msg.velocity = [0.0] * len(self.joint_names)
        joints_msg.effort = [0.0] * len(self.joint_names)
        
        # Publish joint states
        self.joint_states_publisher.publish(joints_msg)

    def dance_active(self, command):
        if command.dance_activate_event:
            if not self.dance_active_state:
                self.dance_active_state = True
            else:
                self.dance_active_state = False
        return True

    def pseudo_dance_active(self, command):
        if command.pseudo_dance_event:
            self.dance_active_state = True

    def step_gait(self, state, command):
        """
        Calculate the desired foot locations for the next timestep.

        Parameters
        ----------
        state : State
            Current robot state containing foot locations and tick count.
        command : Command
            Command message specifying velocities and gait events.

        Returns
        -------
        tuple of (numpy.ndarray, numpy.ndarray)
            new_foot_locations with shape (3, 4) and contact_modes with shape (4,).

        """
        contact_modes = self.gait_controller.contacts(state.ticks)
        new_foot_locations = np.zeros((3, 4))
        for leg_index in range(4):
            contact_mode = contact_modes[leg_index]
            if contact_mode == 1:
                new_location = self.stance_controller.next_foot_location(
                    leg_index, state, command)
            else:
                swing_proportion = (
                    self.gait_controller.subphase_ticks(
                        state.ticks) / self.config.swing_ticks)
                new_location = self.swing_controller.next_foot_location(
                    swing_proportion,
                    leg_index,
                    state,
                    command
                )
            new_foot_locations[:, leg_index] = new_location
        return new_foot_locations, contact_modes

    def command_callback(self, command):
        self.current_command = command
        self.control_loop()

    def control_loop(self):
        """Step the controller forward one timestep."""
        command = self.current_command
        if command is None:
            return

        # self.get_logger().info(f'control_loop command is {command}')

        # Update operating state based on command
        if command.activate_event:
            self.state.behavior_state = self.activate_transition_mapping[self.state.behavior_state]
            self.get_logger().info(
                f'received activate_event new behavior_state is {self.state.behavior_state}')
        elif command.trot_event:
            self.state.behavior_state = self.trot_transition_mapping[self.state.behavior_state]
            self.get_logger().info(
                f'received trot_event new behavior_state is {self.state.behavior_state}')
        elif command.hop_event:
            self.state.behavior_state = self.hop_transition_mapping[self.state.behavior_state]
            self.get_logger().info(
                f'received hop_event new behavior_state is {self.state.behavior_state}')

        # disp.show_state(state.behavior_state)
        self.dance_active(command)
        self.pseudo_dance_active(command)

        if self.state.behavior_state == BehaviorState.TROT:
            if (
                abs(command.horizontal_velocity[0]) < 0.01 and
                abs(command.horizontal_velocity[1]) < 0.01 and
                abs(command.yaw_rate) < 0.01
            ):
                # Stand in default stance at commanded height
                self.state.foot_locations = self.config.stance_at_height(command.height)
            else:
                self.state.foot_locations, contact_modes = self.step_gait(
                    self.state,
                    command,
                )

            # Apply the desired body rotation
            rotated_foot_locations = (
                euler2mat(
                    command.roll, command.pitch, 0.0
                )
                @ self.state.foot_locations
            )

            # Construct foot rotation matrix to compensate for body tilt
            (roll, pitch, yaw) = quat2euler(self.state.quat_orientation)
            correction_factor = 0.8
            max_tilt = 0.4
            roll_compensation = correction_factor * \
                np.clip(-roll, -max_tilt, max_tilt)
            pitch_compensation = correction_factor * \
                np.clip(-pitch, -max_tilt, max_tilt)
            rmat = euler2mat(roll_compensation, pitch_compensation, 0)

            rotated_foot_locations = rmat.T @ rotated_foot_locations

            self.state.joint_angles = self.inverse_kinematics(
                rotated_foot_locations, self.config
            )

        elif self.state.behavior_state == BehaviorState.HOP:
            self.state.foot_locations = self.config.stance_at_height(-0.03)
            self.state.joint_angles = self.inverse_kinematics(
                self.state.foot_locations, self.config
            )

        elif self.state.behavior_state == BehaviorState.FINISHHOP:
            self.state.foot_locations = self.config.stance_at_height(-0.105)
            self.state.joint_angles = self.inverse_kinematics(
                self.state.foot_locations, self.config
            )

        elif self.state.behavior_state == BehaviorState.REST:
            yaw_proportion = command.yaw_rate / self.config.max_yaw_rate
            self.smoothed_yaw += (
                self.config.dt
                * clipped_first_order_filter(
                    self.smoothed_yaw,
                    yaw_proportion * -self.config.max_stance_yaw,
                    self.config.max_stance_yaw_rate,
                    self.config.yaw_time_constant,
                )
            )

            if not self.dance_active_state:
                # Set the foot locations to the default stance plus the
                # standard height
                self.state.foot_locations = self.config.stance_at_height(command.height)

                # Apply the desired body rotation
                rotated_foot_locations = (
                    euler2mat(
                        command.roll,
                        command.pitch,
                        self.smoothed_yaw,
                    )
                    @ self.state.foot_locations
                )
            else:
                location_buf = self.get_2d_foot_locations(command)
                if (abs(command.horizontal_velocity[0]) < 0.01) and \
                    (abs(command.horizontal_velocity[1]) < 0.01 and
                        abs(command.yaw_rate == 0)):
                    self.state.foot_locations = location_buf
                else:
                    self.state.foot_locations, contact_modes = self.step_gait(self.state, command)

                rotated_foot_locations = (
                    euler2mat(
                        command.roll / 57.3,
                        command.pitch / 57.3,
                        command.yaw / 57.3
                    )
                    @ self.state.foot_locations
                )

            # Construct foot rotation matrix to compensate for body tilt
            (roll, pitch, yaw) = quat2euler(self.state.quat_orientation)
            correction_factor = 0.8
            max_tilt = 0.4
            roll_compensation = correction_factor * \
                np.clip(-roll, -max_tilt, max_tilt)
            pitch_compensation = correction_factor * \
                np.clip(-pitch, -max_tilt, max_tilt)
            rmat = euler2mat(roll_compensation, pitch_compensation, 0)

            rotated_foot_locations = rmat.T @ rotated_foot_locations

            self.state.joint_angles = self.inverse_kinematics(
                rotated_foot_locations, self.config
            )

        self.state.ticks += 1
        self.state.pitch = command.pitch
        self.state.roll = command.roll
        self.state.height = command.height

        self.state.joint_angles = self.limit_joint_angles(self.state.joint_angles)

        if self.publish_states:
            self.publish_state()
        if self.publish_joint_control:
            self.publish_joints_command()
        if self.publish_foot_contacts:
            self.publish_foot_contact()
        if self.publish_joint_states:  # Add this line
            self.publish_joint_state()

    def get_2d_foot_locations(self, command):
        location = command.legs_location
        return np.array([location.row1, location.row2, location.row3])

    def limit_joint_angles(self, joint_angles):
        """Adjust joint angles to be within the limits."""
        max_lim = np.array([
            [1.2, 0.6, 1, 0.5],
            [1.3, 1.3, 1.6, 1.6],
            [0.7, 0.7, 0, 0]
        ])
        min_lim = np.array([
            [-0.5, -1, -0.6, -1],
            [0, 0, -0.6, -0.6],
            [-1.5, -1.5, -1.2, -1.2]
        ])
        return np.clip(joint_angles, min_lim, max_lim)

    def publish_state(self):
        state_msg = String()
        state_msg.data = str(self.state.__dict__)
        self.state_publisher.publish(state_msg)

    def publish_joints_command(self):
        joints_cmd_msg = JointTrajectory()
        joints_cmd_msg.header.stamp = self.get_clock().now().to_msg()
        joints_cmd_msg.joint_names = self.joint_names

        point = JointTrajectoryPoint()
        point.positions = convert_to_JTP_positions(self.state.joint_angles)
        point.time_from_start = rclpy.duration.Duration(seconds=1.0 / 60.0).to_msg()

        joints_cmd_msg.points.append(point)
        self.joint_trajectory_publisher.publish(joints_cmd_msg)


def main(args=None):
    rclpy.init(args=args)
    config = Configuration()
    node = StanfordControllerNode(config, four_legs_inverse_kinematics)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Node interrupted by user, shutting down...")
    finally:
        node.destroy_node()


if __name__ == '__main__':
    main()
