#!/usr/bin/env python3

# SPDX-License-Identifier: Apache-2.0
#
# Copyright (c) 2025 MangDang
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (Command, LaunchConfiguration,
                                  PathJoinSubstitution)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

ROBOT_MODEL = os.getenv('ROBOT_MODEL', default='mini_pupper_2')


def get_config():
    bringup_package = get_package_share_directory('mini_pupper_bringup')
    config_file_name = ROBOT_MODEL + '.yaml'
    config_file_path = os.path.join(bringup_package, 'config', config_file_name)

    with open(config_file_path, 'r') as f:
        configuration = yaml.safe_load(f)

    sensors_config = configuration.get('sensors', {})
    sensors_config.setdefault('lidar', False)
    sensors_config.setdefault('imu', False)
    sensors_config.setdefault('camera', False)

    ports_config = configuration.get('ports', {})

    return sensors_config, ports_config


def generate_launch_description():
    bringup_package = FindPackageShare('mini_pupper_bringup')
    description_package = FindPackageShare('mini_pupper_description')

    sensors_config, ports_config = get_config()

    # Convert bool to str because cannot pass bool directly to launch_arguments.
    has_lidar = str(sensors_config['lidar'])
    has_imu = str(sensors_config['imu'])
    has_camera = str(sensors_config['camera'])
    lidar_port = ports_config['lidar']
    has_display = str(ports_config.get('display', False))

    # Disable lidar for now, not supported with Stanford Controller yet.
    has_lidar = 'False'

    use_sim_time = LaunchConfiguration('use_sim_time')
    use_sim_time_launch_arg = DeclareLaunchArgument(
        name='use_sim_time',
        default_value='False',
        description='Use simulation (Gazebo) clock if true'
    )

    hardware_connected = LaunchConfiguration('hardware_connected')
    hardware_connected_launch_arg = DeclareLaunchArgument(
        name='hardware_connected',
        default_value='True',
        description='Set to true if connected to a physical robot'
    )

    description_launch_path = PathJoinSubstitution(
        [description_package, 'launch', 'mini_pupper_description.launch.py']
        # [description_package, 'launch', 'stanford_visualisation.launch.py']
    )
    description_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(description_launch_path),
        launch_arguments={
            'use_sim_time': use_sim_time,
        }.items()
    )

    hardware_interface_launch_path = PathJoinSubstitution(
        [bringup_package, 'launch', 'hardware_interface.launch.py']
    )
    hardware_interface_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(hardware_interface_launch_path),
        condition=IfCondition(hardware_connected),
        launch_arguments={
            'has_lidar': has_lidar,
            'has_imu': has_imu,
            'has_camera': has_camera,
            'lidar_port': lidar_port,
            'has_display': has_display
        }.items()
    )
    
    description_path = PathJoinSubstitution(
        [description_package, 'urdf', ROBOT_MODEL, 'mini_pupper_description.urdf.xacro']
    )

    stanford_controller_launch_path = PathJoinSubstitution(
        [FindPackageShare('stanford_controller'), 'stanford_controller.launch.py']
    )
    stanford_controller_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(stanford_controller_launch_path),
        launch_arguments={
            'orientation_from_imu': has_imu,
            'publish_joint_control': 'True',
            'description_path': description_path,
        }.items()
    )

    joints_config_path = PathJoinSubstitution(
        [description_package, 'config', 'champ', ROBOT_MODEL, 'joints.yaml']
    )
    links_config_path = PathJoinSubstitution(
        [description_package, 'config', 'champ', ROBOT_MODEL, 'links.yaml']
    )
    gait_config_path = PathJoinSubstitution(
        [description_package, 'config', 'champ', ROBOT_MODEL, 'gait.yaml']
    )

    use_sim_time = LaunchConfiguration('use_sim_time')
    use_sim_time_launch_arg = DeclareLaunchArgument(
        name='use_sim_time',
        default_value='False',
        description='Use simulation (Gazebo) clock if true'
    )
    # quadruped_controller = Node(
    #     package='champ_base',
    #     executable='quadruped_controller_node',
    #     output='screen',
    #     parameters=[
    #         {'use_sim_time': use_sim_time},
    #         {'gazebo': use_sim_time},
    #         {'publish_joint_states': True},
    #         {'publish_joint_control': True},
    #         {'publish_foot_contacts': True},
    #         {'joint_controller_topic': 'joint_group_effort_controller/joint_trajectory'},
    #         {'urdf': Command(['xacro ', description_path])},
    #         joints_config_path,
    #         links_config_path,
    #         gait_config_path,
    #     ],
    #     remappings=[('/cmd_vel/smooth', '/cmd_vel')],
    # )

    state_estimator = Node(
        package='champ_base',
        executable='state_estimation_node',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'orientation_from_imu': True},
            {'urdf': Command(['xacro ', description_path])},
            joints_config_path,
            links_config_path,
            gait_config_path,
        ],
    )
    
    ekf_localization_launch_path = PathJoinSubstitution(
        [bringup_package, 'launch', 'ekf_localization.launch.py']
    )
    ekf_localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(ekf_localization_launch_path),
        launch_arguments={
            'use_sim_time': use_sim_time
        }.items()
    )

    return LaunchDescription([
        use_sim_time_launch_arg,
        hardware_connected_launch_arg,
        description_launch,
        hardware_interface_launch,
        stanford_controller_launch,
        ekf_localization_launch,
        state_estimator,
    ])
