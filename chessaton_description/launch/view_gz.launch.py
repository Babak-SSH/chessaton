#!/usr/bin/env -S ros2 launch
"""Visualisation of SDF model for panda in Ignition Gazebo. Note that the generated model://panda/model.sdf descriptor is used."""

from os import path
from typing import List

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description() -> LaunchDescription:
    # Declare all launch arguments
    declared_arguments = generate_declared_arguments()

    # Get substitution for all arguments
    description_package = LaunchConfiguration("description_package")
    description_filepath = LaunchConfiguration("description_filepath")
    moveit_config_package = "chessaton_moveit_config"
    name = LaunchConfiguration("name")
    prefix = LaunchConfiguration("prefix")
    ros2_control = LaunchConfiguration("ros2_control")
    ros2_control_plugin = LaunchConfiguration("ros2_control_plugin")
    ros2_control_command_interface = LaunchConfiguration(
        "ros2_control_command_interface"
    )
    world = LaunchConfiguration("world")
    model = LaunchConfiguration("model")
    rviz_config = LaunchConfiguration("rviz_config")
    use_sim_time = LaunchConfiguration("use_sim_time")
    ign_verbosity = LaunchConfiguration("ign_verbosity")
    log_level = LaunchConfiguration("log_level")
    use_camera = LaunchConfiguration("use_camera")

    # URDF
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare(description_package), description_filepath]
            ),
            " ",
            "name:=",
            name,
            " ",
            "prefix:=",
            prefix,
            " ",
            "ros2_control:=",
            ros2_control,
            " ",
            "ros2_control_plugin:=",
            ros2_control_plugin,
            " ",
            "ros2_control_command_interface:=",
            ros2_control_command_interface,
            " ",
            "model:=",
            model,
            " ",
            "use_camera:=",
            use_camera,
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    # Launch Gazebo
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("gazebo_ros"),
                    "launch",
                    "gazebo.launch.py",
                ]
            )
        ),
                launch_arguments={'world': world}.items(),
    )

    # gazebo = ExecuteProcess(
    #         cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_factory.so',  '-s', 'libgazebo_ros_init.so', world],
    #         output='screen'
    #     )

    # robot state publisher
    node_robot_state_publisher=Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="log",
        arguments=["--ros-args", "--log-level", log_level],
        parameters=[robot_description, {'use_sim_time': use_sim_time}]
    )
    # spawning robot in gazebo
    spawn_entity=Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description',
                   '-entity', 'chessaton'],
        output='screen')

    joint_state_controller_spawner=Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    joint_trajectory_controller_spawner=Node(
        package="controller_manager",
        executable="spawner",
        arguments=["chessaton_arm_controller", "-c", "/controller_manager"],
    )

    chessaton_hand_controller_spawner=Node(
        package="controller_manager",
        executable="spawner",
        arguments=["chessaton_hand_controller", "-c", "/controller_manager"],
    )

    # left_finger_controller_spawner = Node(
    #     package="controller_manager",
    #     executable="spawner",
    #     arguments=["chessaton_finger_left_controller", "-c", "/controller_manager"],
    # )

    # right_finger_controller_spawner = Node(
    #     package="controller_manager",
    #     executable="spawner",
    #     arguments=["chessaton_finger_right_controller", "-c", "/controller_manager"],
    # )
        
    return LaunchDescription(declared_arguments+[

        gazebo, 
        node_robot_state_publisher,
        spawn_entity,

        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=spawn_entity,
                on_exit=[joint_state_controller_spawner],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=joint_state_controller_spawner,
                on_exit=[joint_trajectory_controller_spawner],
            )
        ),
        RegisterEventHandler(
                OnProcessExit(
                    target_action = joint_trajectory_controller_spawner,
                    on_exit = [
                        chessaton_hand_controller_spawner,
                    ]
                )
            ),
    ])

def generate_declared_arguments() -> List[DeclareLaunchArgument]:
    """
    Generate list of all launch arguments that are declared for this launch script.
    """

    return [
        # Locations of robot resources
        DeclareLaunchArgument(
            "description_package",
            default_value="chessaton_description",
            description="Custom package with robot description.",
        ),
        DeclareLaunchArgument(
            "description_filepath",
            default_value=path.join("urdf", "chessaton.urdf.xacro"),
            description="Path to xacro or URDF description of the robot, relative to share of `description_package`.",
        ),
        # Naming of the robot
        DeclareLaunchArgument(
            "name",
            default_value="chessaton",
            description="Name of the robot.",
        ),
        DeclareLaunchArgument(
            "prefix",
            default_value="chessaton_",
            description="Prefix for all robot entities. If modified, then joint names in the configuration of controllers must also be updated.",
        ),
        # ROS 2 control
        DeclareLaunchArgument(
            "ros2_control",
            default_value="true",
            description="Flag to enable ros2 controllers for manipulator.",
        ),
        DeclareLaunchArgument(
            "ros2_control_plugin",
            default_value="gz",
            description="The ros2_control plugin that should be loaded for the manipulator ('fake', 'gz', 'real' or custom).",
        ),
        DeclareLaunchArgument(
            "ros2_control_command_interface",
            default_value="position",
            description="The output control command interface provided by ros2_control ('position', 'velocity', 'effort' or certain combinations 'position,velocity').",
        ),
        # World and model for Gazebo
        DeclareLaunchArgument(
            "world",
            default_value=path.join("world", "chessaton.world"),
            description="Name or filepath of world to load.",
        ),
        DeclareLaunchArgument(
            "model",
            default_value="chessaton",
            description="Name or filepath of model to load.",
        ),
        # Miscellaneous
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="true",
            description="If true, use simulated clock.",
        ),
        DeclareLaunchArgument(
            "log_level",
            default_value="warn",
            description="The level of logging that is applied to all ROS 2 nodes launched by this script.",
        ),
        DeclareLaunchArgument(
            "use_camera",
            default_value="false",
            description="If true, add camera to scene",
        ),
    ]