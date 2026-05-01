from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    enable_lidar = LaunchConfiguration("enable_lidar")
    enable_gps = LaunchConfiguration("enable_gps")

    return LaunchDescription([
        DeclareLaunchArgument(
            "enable_lidar",
            default_value="false",
            description="Launch the RPLIDAR C1 node alongside the robot node.",
        ),
        DeclareLaunchArgument(
            "enable_gps",
            default_value="true",
            description="Launch the robot_gps node alongside the robot node.",
        ),
        Node(
            package="bridge",
            executable="bridge",
            name="bridge",
            output="screen",
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution(
                    [FindPackageShare("rplidar_ros"), "launch", "rplidar_c1.launch.py"]
                )
            ),
            condition=IfCondition(enable_lidar),
            launch_arguments={
                "serial_port": "/dev/rplidar",
                "serial_baudrate": "460800",
                "frame_id": "laser_frame",
                "topic_name": "scan",
                "scan_mode": "Standard",
                "angle_compensate": "true",
            }.items(),
        ),
        Node(
            package="robot",
            executable="robot",
            name="robot",
            output="screen",
        ),
        Node(
            package="sensors",
            executable="robot_gps",
            name="robot_gps",
            output="screen",
            condition=IfCondition(enable_gps),
        ),
    ])
