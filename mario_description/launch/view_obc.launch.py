#!/usr/bin/env -S ros2 launch
"""Visualisation of URDF model for MARIO in RViz2"""

from os import path
from typing import List

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    # ------------------------------------------------ #
    # Launch Arguments
    # ------------------------------------------------ #
    declared_arguments = generate_declared_arguments()

    collision_arm = LaunchConfiguration("collision_arm")
    ros2_control = LaunchConfiguration("ros2_control")
    ros2_control_plugin = LaunchConfiguration("ros2_control_plugin")
    gazebo_preserve_fixed_joint = LaunchConfiguration("gazebo_preserve_fixed_joint")
    use_sim_time = LaunchConfiguration("use_sim_time")
    log_level = LaunchConfiguration("log_level")

    # ------------------------------------------------ #
    # Config
    # ------------------------------------------------ #

    # Description Package
    description_package = "mario_description"

    # URDF
    description_filepath = path.join("urdf", "mario.urdf.xacro")

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare(description_package), description_filepath]
            ),
            " ",
            "collision_arm:=",
            collision_arm,
            " ",
            "ros2_control:=",
            ros2_control,
            " ",
            "ros2_control_plugin:=",
            ros2_control_plugin,
            " ",
            "gazebo_preserve_fixed_joint:=",
            gazebo_preserve_fixed_joint,
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    # Rviz Config
    rviz_config = path.join(
        get_package_share_directory("mario_description"),
        "rviz",
        "view.rviz",
    )

    # ------------------------------------------------ #
    # Nodes
    # ------------------------------------------------ #
    nodes = [

        Node(
            package="rviz2",
            executable="rviz2",
            output="log",
            arguments=[
                "--display-config",
                rviz_config,
                "--ros-args",
                "--log-level",
                log_level,
            ],
            parameters=[{"use_sim_time": use_sim_time}],
        ),
    ]

    return LaunchDescription(declared_arguments + nodes)


def generate_declared_arguments() -> List[DeclareLaunchArgument]:
    """
    Generate list of all launch arguments that are declared for this launch script.
    """

    return [
        DeclareLaunchArgument(
            "collision_arm",
            default_value="true",
            description="Flag to enable collision geometry for manipulator's arm.",
        ),
        DeclareLaunchArgument(
            "ros2_control",
            default_value="true",
            description="Flag to enable ros2 controllers for manipulator.",
        ),
        DeclareLaunchArgument(
            "ros2_control_plugin",
            default_value="fake",
            description="The ros2_control plugin that should be loaded for the manipulator ('fake', 'gz')",
        ),
        DeclareLaunchArgument(
            "gazebo_preserve_fixed_joint",
            default_value="false",
            description="Flag to preserve fixed joints and prevent lumping when generating SDF for Gazebo.",
        ),
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
            description="If true, use simulated clock.",
        ),
        DeclareLaunchArgument(
            "log_level",
            default_value="warn",
            description="The level of logging that is applied to all ROS 2 nodes launched by this script.",
        ),
    ]