#!/usr/bin/env python3
#
# Copyright 2024 ROBOTIS CO., LTD.
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
#
# Author: Wonho Yun, Sungho Woo, Woojin Wie
import os
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import RegisterEventHandler
from launch.conditions import IfCondition
from launch.conditions import UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command
from launch.substitutions import FindExecutable
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    # Declare launch arguments
    declared_arguments = [
        DeclareLaunchArgument(
            'start_rviz', default_value='false', description='Whether to execute rviz2'
        ),
        DeclareLaunchArgument(
            'prefix',
            default_value='""',
            description='Prefix of the joint and link names',
        ),
        DeclareLaunchArgument(
            'use_sim',
            default_value='false',
            description='Start robot in Gazebo simulation.',
        ),
        DeclareLaunchArgument(
            'use_mock_hardware',
            default_value='false',
            description='Use mock hardware mirroring command.',
        ),
        DeclareLaunchArgument(
            'mock_sensor_commands',
            default_value='false',
            description='Enable mock sensor commands.',
        ),
        DeclareLaunchArgument(
            'port_name',
            default_value='/dev/ttyUSB0',
            description='Port name for hardware connection.',
        ),
        DeclareLaunchArgument(
            'init_position',
            default_value='false',
            description='Whether to launch the init_position node',
        ),
        DeclareLaunchArgument(
            'ros2_control_type',
            default_value='hvl_manipulator_position',
            description='Type of ros2_control',
        ),
        DeclareLaunchArgument(
            'init_position_file',
            default_value='initial_positions.yaml',
            description='Path to the initial position file',
        ),
        DeclareLaunchArgument(
            'warehouse_sqlite_path',
            default_value=os.path.expanduser('~/.ros/warehouse_ros.sqlite'),
            description='Path where the warehouse database should be stored',
        ),
        DeclareLaunchArgument(
            'publish_robot_description_semantic',
            default_value='true',
            description='Whether to publish robot description semantic',
        ),
    ]


    # Launch configurations
    start_rviz = LaunchConfiguration('start_rviz')
    prefix = LaunchConfiguration('prefix')
    use_sim = LaunchConfiguration('use_sim')
    use_mock_hardware = LaunchConfiguration('use_mock_hardware')
    mock_sensor_commands = LaunchConfiguration('mock_sensor_commands')
    port_name = LaunchConfiguration('port_name')
    init_position = LaunchConfiguration('init_position')
    ros2_control_type = LaunchConfiguration('ros2_control_type')
    init_position_file = LaunchConfiguration('init_position_file')
    warehouse_sqlite_path = LaunchConfiguration('warehouse_sqlite_path')
    publish_robot_description_semantic = LaunchConfiguration('publish_robot_description_semantic')

    # Generate URDF file using xacro
    urdf_file = Command([
        PathJoinSubstitution([FindExecutable(name='xacro')]),
        ' ',
        PathJoinSubstitution([
            FindPackageShare('hvl_manipulator_description'),
            'urdf',
            'hvl_manipulator',
            'hvl_manipulator.urdf.xacro',
        ]),
        ' ',
        'prefix:=',
        prefix,
        ' ',
        'use_sim:=',
        use_sim,
        ' ',
        'use_mock_hardware:=',
        use_mock_hardware,
        ' ',
        'mock_sensor_commands:=',
        mock_sensor_commands,
        ' ',
        'port_name:=',
        port_name,
        ' ',
        'ros2_control_type:=',
        ros2_control_type,
    ])

    # Paths for configuration files
    controller_manager_config = PathJoinSubstitution([
        FindPackageShare('hvl_manipulator_bringup'),
        'config',
        'hvl_manipulator',
        'hardware_controller_manager.yaml',
    ])
    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare('hvl_manipulator_moveit_config'), 'config', 'moveit.rviz']
    )
    trajectory_params_file = PathJoinSubstitution([
        FindPackageShare('hvl_manipulator_bringup'),
        'config',
        'hvl_manipulator',
        init_position_file,
    ])

    moveit_config = (
        MoveItConfigsBuilder(
            robot_name='hvl_manipulator', package_name='hvl_manipulator_moveit_config')
        .robot_description_semantic(
            str(Path('config') / 'hvl_manipulator' / 'hvl_manipulator.srdf'))
        .joint_limits(str(Path('config') / 'hvl_manipulator' / 'joint_limits.yaml'))
        .trajectory_execution(
            str(Path('config') / 'hvl_manipulator' / 'moveit_controllers.yaml'))
        .robot_description_kinematics(
            str(Path('config') / 'hvl_manipulator' / 'kinematics.yaml'))
        .to_moveit_configs()
    )
    warehouse_ros_config = {
        'warehouse_plugin': 'warehouse_ros_sqlite::DatabaseConnection',
        'warehouse_host': warehouse_sqlite_path,
    }
    # Define nodes
    control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[{'robot_description': urdf_file}, controller_manager_config],
        output='both',
        condition=UnlessCondition(use_sim),
    )

    robot_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'arm_controller',
            'joint_state_broadcaster',
        ],
        output='both',
        parameters=[{'robot_description': urdf_file}],
    )

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': urdf_file, 'use_sim_time': use_sim}],
        output='both',
    )

    joint_trajectory_executor = Node(
        package='hvl_manipulator_bringup',
        executable='joint_trajectory_executor',
        parameters=[trajectory_params_file],
        output='both',
        condition=IfCondition(init_position),
    )
    move_group_node = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        output='both',
        parameters=[
            moveit_config.to_dict(),
            warehouse_ros_config,
            {
                'use_sim_time': use_sim,
                'publish_robot_description_semantic': publish_robot_description_semantic,
            },
        ],
    )

    rviz_node = Node(
        package='rviz2',
        condition=IfCondition(start_rviz),
        executable='rviz2',
        name='rviz2_moveit',
        output='log',
        arguments=['-d', rviz_config_file],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
            warehouse_ros_config,
            {
                'use_sim_time': use_sim,
            },
        ],
    )

    # Event handlers to ensure order of execution
    delay_rviz_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=robot_controller_spawner, on_exit=[rviz_node]
        )
    )

    delay_joint_trajectory_executor_after_controllers = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=robot_controller_spawner,
            on_exit=[joint_trajectory_executor],
        )
    )

    return LaunchDescription(
        declared_arguments
        + [
            control_node,
            move_group_node,
            robot_controller_spawner,
            robot_state_publisher_node,
            delay_rviz_after_joint_state_broadcaster_spawner,
            delay_joint_trajectory_executor_after_controllers,
        ]
    )
