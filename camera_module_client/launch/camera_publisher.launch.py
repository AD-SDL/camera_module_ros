from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    launch_d = LaunchDescription()

    camera_module_client = Node(
            package = 'camera_module_client',
            namespace = 'std_ns',
            executable = 'camera_publisher',
            output = "screen",
            name='CameraPublisher1'
    )

    launch_d.add_action(camera_module_client)
    return launch_d