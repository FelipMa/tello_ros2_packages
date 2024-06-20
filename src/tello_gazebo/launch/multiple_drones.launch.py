"""Simulate multiple Tello drones"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration

def launch_setup(context):
    formation = LaunchConfiguration('formation').perform(context)

    if formation == 'square':
        drones = [0] * 4
    else:
        drones = [0] * 5

    tello_gazebo_path = get_package_share_directory('tello_gazebo')
    tello_description_path = get_package_share_directory('tello_description')

    world_path = os.path.join(tello_gazebo_path, 'worlds', 'simple.world')

    # Global entities
    entities = [
        # Launch Gazebo, loading tello.world
        ExecuteProcess(cmd=[
            'gazebo',
            '--verbose',
            '-s', 'libgazebo_ros_init.so',  # Publish /clock
            '-s', 'libgazebo_ros_factory.so',  # Provide gazebo_ros::Node
            world_path
        ], output='screen')
    ]

    if formation == 'square':
        starting_positions = [
            ["2", "2"],
            ["-2", "2"],
            ["-2", "-2"],
            ["2", "-2"]
        ]
    else:
        starting_positions = [
            ["0", "0"],
            ["2", "5"],
            ["3", "3"],
            ["4", "-1"],
            ["-3", "5"]
        ]

    # Per-drone entities
    for idx, namespace in enumerate(drones):
        suffix = '_' + str(idx + 1)
        urdf_path = os.path.join(
            tello_description_path, 'urdf', 'tello' + suffix + '.urdf')

        entities.extend([
            # Add a drone to the simulation
            Node(package='tello_gazebo', executable='inject_entity.py', output='screen',
                 arguments=[urdf_path, starting_positions[idx][0], starting_positions[idx][1], '1', '0', "simu_tello" + str(idx + 1)]),
        ])

    return entities


def generate_launch_description():
    formation_arg = DeclareLaunchArgument(
        'formation',
        default_value='default',
        description='Choose formation setup (Options: default, square)'
    )

    return LaunchDescription([formation_arg, OpaqueFunction(function=launch_setup)])
