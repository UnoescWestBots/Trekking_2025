import os
from time import time

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node

def generate_launch_description():

    package_name='traxxas'

    rsp = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(package_name),'launch','rsp.launch.py'
                )]), launch_arguments={'use_sim_time': 'true'}.items()
    )

    # Use get_package_share_directory to get the correct path for the world file
    world_file = os.path.join(
                    get_package_share_directory('traxxas'), 'worlds', 'empty.world'
    )  

    # Include the Gazebo launch file, provided by the gazebo_ros package
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py'
            )]),
        launch_arguments={'world': world_file}.items()
    )

    # Run the spawner node from the gazebo_ros package. The entity name doesn't really matter if you only have a single robot.
    # Add a timer to allow Gazebo to start properly before spawning the robot
    spawn_entity = TimerAction(
        period=1.0,  # wait for 1 seconds
        actions=[
            Node(
                package='ros_gz_sim',
                executable='create',
                arguments=['-topic', '/robot_description', '-entity', 'traxxas'],
                output='screen'
            )
        ]
    )

    # Launch them all!
    return LaunchDescription([
        rsp,
        gazebo,
        spawn_entity,
    ])
