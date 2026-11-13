#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")

    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="true"),

        # RViz to watch the map being built
        IncludeLaunchDescription(
            PathJoinSubstitution([FindPackageShare("asl_tb3_sim"), "launch", "rviz.launch.py"]),
            launch_arguments={"use_sim_time": use_sim_time}.items(),
        ),

        # publishes /state from TF
        Node(
            package="asl_tb3_lib",
            executable="state_publisher.py",
            parameters=[{"use_sim_time": use_sim_time}],
        ),

        # your navigator from HW2
        Node(
            package="autonomy_repo",
            executable="navigator.py",
            parameters=[{"use_sim_time": use_sim_time}],
        ),

        # frontier explorer, delayed so the map and navigator are up first
        TimerAction(
            period=5.0,
            actions=[
                Node(
                    package="autonomy_repo",
                    executable="frontier_explorer.py",
                    parameters=[{"use_sim_time": use_sim_time}],
                ),
            ],
        ),
    ])