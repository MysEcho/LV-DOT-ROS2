# LV-DOT dynamic obstacle detector launch (ROS2)
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config_arg = DeclareLaunchArgument(
        'config',
        default_value=PathJoinSubstitution([
            FindPackageShare('onboard_detector'), 'cfg', 'detector_param.yaml'
        ]),
        description='Path to the detector parameter file',
    )

    detector_node = Node(
        package='onboard_detector',
        executable='detector_node',
        name='onboard_detector',
        output='screen',
        parameters=[LaunchConfiguration('config')],
    )

    # NOTE: the YOLO color-image detector (scripts/yolo_detector) is not required in
    # lidar_only mode and has not been ported to ROS2/rclpy yet.

    return LaunchDescription([config_arg, detector_node])
