from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='tello_py_driver',
            executable='tello_ros2_driver.py',
            name='tello_py_driver',
            namespace='tello_ns_2',
            parameters=[
                {'network_interface': "wlx74da384173d1"}]
        )
    ])