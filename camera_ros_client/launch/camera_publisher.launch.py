
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    camera_name = LaunchConfiguration('camera_name')
    camera_number = LaunchConfiguration('camera_number')

    declare_use_camera_name_cmd = DeclareLaunchArgument(
        name = "camera_name",
        default_value='CameraPublisher1',
        description='Flag to accept camera name')

    declare_use_camera_number_cmd = DeclareLaunchArgument(
        name = 'camera_number',
        default_value="0",
        description='Flag to accept camera number')

    camera_ros_client = Node(
            package = 'camera_ros_client',
            namespace = 'std_ns',
            executable = 'camera_publisher',
            output = "screen",
            name = camera_name,
            parameters = [{"camera_number":camera_number}],
            emulate_tty=True

    )
    launch_d = LaunchDescription()

    launch_d.add_action(declare_use_camera_name_cmd)
    launch_d.add_action(declare_use_camera_number_cmd)
    launch_d.add_action(camera_ros_client)
    return launch_d