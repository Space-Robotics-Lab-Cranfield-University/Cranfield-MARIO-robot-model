#!/usr/bin/env -S ros2 launch
"""Visualisation of URDF model for MARIO in Gazebo"""

from os import path
from typing import List

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)

from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    # ------------------------------------------------ #
    # Launch Arguments
    # ------------------------------------------------ #
    declared_arguments = generate_declared_arguments()

    description_package = LaunchConfiguration("description_package")
    description_filepath = LaunchConfiguration("description_filepath")
    world = LaunchConfiguration("world")
    use_sim_time = LaunchConfiguration("use_sim_time")
    gz_verbosity = LaunchConfiguration("gz_verbosity")
    log_level = LaunchConfiguration("log_level")

    # ------------------------------------------------ #
    # Config
    # ------------------------------------------------ #

    # URDF
    _robot_description_xml = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare(description_package), description_filepath]
            ),
        ]
    )
    robot_description = {"robot_description": _robot_description_xml}

    # ------------------------------------------------ #
    # Launch Descriptions
    # ------------------------------------------------ #

    launch_descriptions = [
        # Launch Gazebo
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution(
                    [
                        FindPackageShare("ros_gz_sim"),
                        "launch",
                        "gz_sim.launch.py",
                    ]
                )
            ),
            launch_arguments=[("gz_args", [world, " -v ", gz_verbosity])],
        ),
    ]

    # ------------------------------------------------ #
    # ROS Nodes
    # ------------------------------------------------ #

    # List of nodes to be launched
    nodes = [
        # robot_state_publisher
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            output="screen",
            arguments=["--ros-args", "--log-level", log_level],
            parameters=[
                robot_description,
                {
                    "publish_frequency": 50.0,
                    "frame_prefix": "",
                    "use_sim_time": use_sim_time,
                },
            ],
        ),
        # ros_gz_sim
        Node(
            package="ros_gz_sim",
            executable="create",
            output="screen",
            arguments=[
                "-topic",
                "robot_description",
                "--ros-args",
                "--log-level",
                log_level,
            ],
            parameters=[{"use_sim_time": use_sim_time}],
        ),
    ]

    return LaunchDescription(declared_arguments + launch_descriptions + nodes)


def generate_declared_arguments() -> List[DeclareLaunchArgument]:
    """
    Generate list of all launch arguments that are declared for this launch script.
    """

    return [
        DeclareLaunchArgument(
            "description_package",
            default_value="mario_description",
            description="Custom package with robot description.",
        ),
        DeclareLaunchArgument(
            "description_filepath",
            default_value=path.join("urdf", "mario.urdf.xacro"),
            description="Path to xacro or URDF description of the robot",
        ),
        DeclareLaunchArgument(
            "world",
            default_value="worlds/simple_lab.sdf",
            description="Name or filepath of world to load.",
        ),
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="true",
            description="If true, use simulated clock.",
        ),
        DeclareLaunchArgument(
            "gz_verbosity",
            default_value="3",
            description="Verbosity level for Gazebo (0~4).",
        ),
        DeclareLaunchArgument(
            "log_level",
            default_value="warn",
            description="The level of logging that is applied to all ROS 2 nodes launched by this script.",
        ),
    ]
