from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='winners_pkg',
            namespace='ballPose',
            executable='ballPose',
            name='sim'
        ),
        Node(
            package='winners_pkg',
            namespace='velcroPadPose',
            executable='velcroPadPose',
            name='sim'
        ),
        Node(
            package='winners_pkg',
            namespace='RobotControl',
            executable='RobotControl',
            name='sim'
        ),
        Node(
            package='winners_pkg',
            namespace='Brain',
            executable='Brain',
            name='sim'
        ),
    ])