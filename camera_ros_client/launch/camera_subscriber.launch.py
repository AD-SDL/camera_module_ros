
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    launch_d = LaunchDescription()

    camera_name = LaunchConfiguration('camera_name')

    declare_use_camera_name_cmd = DeclareLaunchArgument(
        name = "camera_name",
        default_value='CameraSubscriber1',
        description='Flag to accept camera name')

    camera_ros_client = Node(
            package = 'camera_ros_client',
            namespace = 'std_ns',
            executable = 'camera_subscriber',
            output = "screen",
            name='CameraSubscriber',
            parameters = [{"camera_name":camera_name}],
            emulate_tty=True

    )

    launch_d.add_action(declare_use_camera_name_cmd)
    launch_d.add_action(camera_ros_client)
    return launch_d
