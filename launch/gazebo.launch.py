import os
import xacro

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource


def launch_setup(context, *args, **kwargs):
    # constants
    pkg_shared_directory = get_package_share_directory('unitree_go1_gazebo')

    xacro_file = os.path.join(pkg_shared_directory, 'xacro', 'robot.xacro')
    rviz_config_file = os.path.join(pkg_shared_directory, "rviz", "go1_model.rviz")
    gz_bridge_config_file = os.path.join(pkg_shared_directory, 'config', 'gz_bridge.yaml')
    world_file = os.path.join(pkg_shared_directory, 'worlds', 'empty.sdf')

    # variables
    init_height = context.launch_configurations['height']
    robot_description = (
        xacro.process_file(xacro_file, mappings={
            'DEBUG': 'false',
            'GAZEBO': 'true'
        }).toxml())

    # create nodes
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz_ocs2',
        output='screen',
        arguments=["-d", rviz_config_file]
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[
            {
                'publish_frequency': 20.0,
                'use_tf_static': True,
                'robot_description': robot_description,
                'ignore_timestamp': True
            }
        ],
    )

    joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster",
                   "--controller-manager", "/controller_manager"],
    )

    imu_sensor_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["imu_sensor_broadcaster",
                   "--controller-manager", "/controller_manager"]
    )

    force_torque_sensor_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["fr_ft_sensor_broadcaster",
                     "--controller-manager", "/controller_manager"]
    )

    gz_spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=['-topic', 'robot_description', '-name',
                   'robot', '-allow_renaming', 'true', '-z', init_height],
    )

    leg_joint_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["leg_joint_controller",
                   "--controller-manager", "/controller_manager"],
    )

    unitree_sdk2_adapter = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["unitree_sdk2_adapter",
                   "--controller-manager", "/controller_manager"],
    )

    gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{
            'config_file': gz_bridge_config_file,
            'qos_overrides./tf_static.publisher.durability': 'transient_local',
        }],
        output='screen'
    )

    gz_spawn_sdf = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [PathJoinSubstitution([FindPackageShare('ros_gz_sim'),
                                   'launch',
                                   'gz_sim.launch.py'])]),
        # launch_arguments=[('gz_args', [' -r -v 4 empty.sdf'])]
        launch_arguments=[('gz_args', [' -r -v 4 ', world_file])]
    )

    return [
        rviz,
        robot_state_publisher,
        # gazebo
        gz_bridge,
        gz_spawn_sdf,
        gz_spawn_robot,
        # ros2_control
        joint_state_broadcaster,
        imu_sensor_broadcaster,
        force_torque_sensor_broadcaster,
        leg_joint_controller
        # RegisterEventHandler(
        #     event_handler=OnProcessExit(
        #         target_action=leg_pd_controller,
        #         on_exit=[imu_sensor_broadcaster, joint_state_broadcaster, unitree_sdk2_adapter],
        #     )
        # ),
    ]


def generate_launch_description():
    height = DeclareLaunchArgument(
        'height',
        default_value='0.5',
        description='Init height in simulation'
    )

    return LaunchDescription([
        height,
        OpaqueFunction(function=launch_setup),
    ])
