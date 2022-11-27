from http.server import executable
from site import execusercustomize
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='action_srvcli',
            executable='action_server',
            name='action_server',
        ),
        None(
            package='action_srvcli',
            executable='action_client',
            name='action_client',
        )
    ])