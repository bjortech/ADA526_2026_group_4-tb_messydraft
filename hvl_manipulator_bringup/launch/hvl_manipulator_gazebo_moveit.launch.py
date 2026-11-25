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

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.actions import RegisterEventHandler
from launch.actions import SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.substitutions import PathJoinSubstitution
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder

import xacro


def generate_launch_description():
    # Launch Arguments
    hvl_manipulator_description_path = os.path.join(
        get_package_share_directory('hvl_manipulator_description')
    )

    hvl_manipulator_bringup_path = os.path.join(
        get_package_share_directory('hvl_manipulator_bringup')
    )

    # Set gazebo sim resource path
    gazebo_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=[
            os.path.join(hvl_manipulator_bringup_path, 'worlds'),
            ':' + str(Path(hvl_manipulator_description_path).parent.resolve()),
        ],
    )

    arguments = LaunchDescription([
        DeclareLaunchArgument(
            'start_rviz', default_value='true', description='Whether to execute rviz2'
        ),
        DeclareLaunchArgument(
            'world', default_value='empty_world', description='Gz sim World'
        ),
        DeclareLaunchArgument(
            'use_sim',
            default_value='true',
            description='Start robot in Gazebo simulation.',
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
    ])

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch'),
            '/gz_sim.launch.py',
        ]),
        launch_arguments=[
            ('gz_args', [LaunchConfiguration('world'), '.sdf', ' -v 1', ' -r'])
        ],
    )

    xacro_file = os.path.join(
        hvl_manipulator_description_path,
        'urdf',
        'hvl_manipulator',
        'hvl_manipulator.urdf.xacro',
    )

    doc = xacro.process_file(xacro_file, mappings={'use_sim': 'true'})

    robot_desc = doc.toprettyxml(indent='  ')

    params = {'robot_description': robot_desc}
    warehouse_sqlite_path = LaunchConfiguration('warehouse_sqlite_path')
    publish_robot_description_semantic = LaunchConfiguration('publish_robot_description_semantic')
    use_sim = LaunchConfiguration('use_sim')
    start_rviz = LaunchConfiguration('start_rviz')

    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params],
    )

    gz_spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-string',
            robot_desc,
            '-x',
            '0.0',
            '-y',
            '0.0',
            '-z',
            '0.0',
            '-R',
            '0.0',
            '-P',
            '0.0',
            '-Y',
            '0.0',
            '-name',
            'hvl_manipulator',
            '-allow_renaming',
            'true',
            '-use_sim',
            'true',
        ],
    )
    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare('hvl_manipulator_moveit_config'), 'config', 'moveit.rviz']
    )
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
    # Controller spawner nodes
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'joint_state_broadcaster',
            '--controller-manager',
            '/controller_manager',
        ],
        output='screen',
    )

    arm_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['arm_controller'],
        output='screen',
    )

    gripper_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['gripper_controller'],
        output='screen',
    )

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        output='screen',
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
            target_action=joint_state_broadcaster_spawner, on_exit=[rviz_node]
        )
    )
    # rviz_config_file = os.path.join(
    #     hvl_manipulator_description_path, 'rviz', 'hvl_manipulator.rviz'
    # )

    # rviz = Node(
    #     package='rviz2',
    #     executable='rviz2',
    #     name='rviz2',
    #     output='log',
    #     arguments=['-d', rviz_config_file],
    # )

    return LaunchDescription([
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=gz_spawn_entity,
                on_exit=[joint_state_broadcaster_spawner],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=joint_state_broadcaster_spawner,
                on_exit=[arm_controller_spawner, gripper_controller_spawner],
            )
        ),
        bridge,
        gazebo_resource_path,
        arguments,
        gazebo,
        node_robot_state_publisher,
        gz_spawn_entity,
        delay_rviz_after_joint_state_broadcaster_spawner,
        # rviz,
    ])
