import os
import xacro

import launch_ros
from launch_ros.actions import Node

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch.conditions import IfCondition


def generate_launch_description():
    # constants
    this_pkg_name = "unitree_go1_gazebo"
    pkg_share_directory = launch_ros.substitutions.FindPackageShare(package=this_pkg_name).find(this_pkg_name)
    xacro_file = os.path.join(pkg_share_directory, "xacro", "robot.xacro")
    rviz_config_file = os.path.join(pkg_share_directory, 'rviz', 'go1_model.rviz')

    # declare arguments
    declare_use_sim_time = DeclareLaunchArgument("use_sim_time", default_value="false",
                                                 description="Use simulation (Gazebo) clock if true")
    declare_start_rviz = DeclareLaunchArgument('start_rviz', default_value='true', description='Start RViz.')

    # process xacro file
    robot_description = (
        xacro.process_file(xacro_file, mappings={
            'DEBUG': 'false',
            'GAZEBO': 'true'
        }).toxml())

    # create nodes
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",

        parameters=[
            {"robot_description": robot_description},
            {"use_tf_static": False},
            {"publish_frequency": 200.0},
            {"ignore_timestamp": True},
            {'use_sim_time': LaunchConfiguration("use_sim_time")}
        ],
        # remappings=(("robot_description", "robot_description")),
    )

    joint_state_publisher_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui"
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config_file],
        condition=IfCondition(LaunchConfiguration('start_rviz'))
    )

    return LaunchDescription(
        [
            declare_use_sim_time,
            declare_start_rviz,
            robot_state_publisher_node,
            joint_state_publisher_node,
            rviz_node
        ]
    )
