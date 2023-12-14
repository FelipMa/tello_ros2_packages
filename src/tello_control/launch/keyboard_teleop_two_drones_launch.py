from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='tello_control',
            executable='keyboard_teleop.py',
            name='keyboard_teleop',
            namespace='tello_ns_1',
            parameters=[
                {'ns': 'tello_ns_1'}]
        ),
        Node(
            package='tello_control',
            executable='keyboard_teleop.py',
            name='keyboard_teleop',
            namespace='tello_ns_2',
            parameters=[
                {'ns': 'tello_ns_2'}]
        )
    ])