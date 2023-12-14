from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Change your network interface according to your network adapter
        Node(
            package='tello_py_driver',
            executable='tello_ros2_driver.py',
            name='tello_py_driver',
            namespace='tello_ns_1',
            parameters=[
                {'network_interface': 'wlp4s0', "stream_on": "False"}]
        ),
        Node(
            package='tello_py_driver',
            executable='tello_ros2_driver.py',
            name='tello_py_driver',
            namespace='tello_ns_2',
            parameters=[
                {'network_interface': 'wlx74da384173d1', "stream_on": "False"}]
        )
    ])