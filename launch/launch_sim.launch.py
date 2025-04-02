import os
from time import time

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, SetEnvironmentVariable, 
                            IncludeLaunchDescription, TimerAction, SetLaunchConfiguration)
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    # Define package paths
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    pkg_traxxas = get_package_share_directory('traxxas')  

    # Gazebo launch file path
    gz_launch_path = PathJoinSubstitution([pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py'])
    gz_model_path = PathJoinSubstitution([pkg_traxxas, 'models'])

    # World file path
    world_file = os.path.join(pkg_traxxas, 'worlds', 'empty.world')

    # Include the robot state publisher launch file
    rsp = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('traxxas'), 'launch', 'rsp.launch.py'
                )]), launch_arguments={'use_sim_time': 'true'}.items()
    )

    # Include the Gazebo launch file with world argument
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([gz_launch_path]),
        launch_arguments={
            'gz_args': [PathJoinSubstitution([pkg_traxxas, 'worlds',
                                              LaunchConfiguration('world_file')])],
            'on_exit_shutdown': 'True'
        }.items(),
    )

    # Timer to wait for Gazebo to start and then spawn the robot
    spawn_entity = TimerAction(
        period=1.0,  # wait for 1 second
        actions=[
            Node(
                package='ros_gz_sim',
                executable='create',
                arguments=['-topic', '/robot_description', '-entity', 'traxxas'],
                output='screen'
            )
        ]
    )

    ign_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="ign_bridge",
        arguments=[
            "/clock" + "@rosgraph_msgs/msg/Clock" + "[ignition.msgs.Clock",
            "/cmd_vel" + "@geometry_msgs/msg/Twist" + "@ignition.msgs.Twist",
            "/tf" + "@tf2_msgs/msg/TFMessage" + "[ignition.msgs.Pose_V",
            "/odom" + "@nav_msgs/msg/Odometry" + "[ignition.msgs.Odometry",
        ],
        output="screen",
    )

    # Return the combined launch description
    return LaunchDescription([
        DeclareLaunchArgument(
            'world',
            default_value='empty',
            description='World to load into Gazebo'
        ),
        SetLaunchConfiguration(name='world_file', 
                               value=[LaunchConfiguration('world'), 
                                      TextSubstitution(text='.world')]),
        SetEnvironmentVariable('IGN_GAZEBO_RESOURCE_PATH', gz_model_path),
        rsp,
        gazebo,
        spawn_entity,
        ign_bridge
    ])
