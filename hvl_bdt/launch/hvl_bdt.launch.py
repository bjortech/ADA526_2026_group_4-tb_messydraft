from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    start_camera = LaunchConfiguration('start_camera')
    params = PathJoinSubstitution([FindPackageShare('hvl_bdt'), 'config', 'params.yaml'])

    realsense = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare('realsense2_camera'), 'launch', 'rs_launch.py'])),
        condition=IfCondition(start_camera),
        launch_arguments={
            'align_depth.enable': 'true',       # depth registered to the colour image
            'pointcloud.enable': 'true',        # coloured PointCloud2
            'pointcloud.stream_filter': '2',    # texture the cloud with the colour stream
            'enable_sync': 'true',
            'rgb_camera.color_profile': '640,480,30',
            'depth_module.depth_profile': '640,480,30',
            'spatial_filter.enable': 'true',
            'temporal_filter.enable': 'true',
        }.items(),
    )

    detector = Node(
        package='hvl_bdt',
        executable='hvl_bdt_node',
        name='hvl_bdt_node',
        parameters=[params],
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument('start_camera', default_value='false',
                              description='Also start the RealSense driver'),
        realsense,
        detector,
    ])
