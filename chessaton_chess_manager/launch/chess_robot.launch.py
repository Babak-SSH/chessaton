#!/usr/bin/env -S ros2 launch
from os import path
from typing import List

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    RegisterEventHandler,
    TimerAction,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
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
    gazebo_sim = LaunchConfiguration("gazebo_sim")
    world = LaunchConfiguration("world")
    model = LaunchConfiguration("model")
    gazebo_preserve_fixed_joint = LaunchConfiguration("gazebo_preserve_fixed_joint")
    enable_servo = LaunchConfiguration("enable_servo")
    enable_rviz = LaunchConfiguration("enable_rviz")
    rviz_config = LaunchConfiguration("rviz_config")
    use_sim_time = LaunchConfiguration("use_sim_time")
    log_level = LaunchConfiguration("log_level")
    use_camera = LaunchConfiguration("use_camera")
    robot_program = LaunchConfiguration("robot_program")

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
            "gazebo_preserve_fixed_joint:=",
            gazebo_preserve_fixed_joint,
            " ",
            "use_camera:=",
            use_camera,
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    # SRDF
    robot_description_semantic_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [
                    FindPackageShare(moveit_config_package),
                    "srdf",
                    "chessaton.srdf.xacro",
                ]
            ),
            " ",
            "name:=",
            name,
            " ",
            "prefix:=",
            prefix,
        ]
    )
    robot_description_semantic = {
        "robot_description_semantic": robot_description_semantic_content
    }

    # Kinematics
    _robot_description_kinematics_yaml = load_yaml(
        moveit_config_package, path.join("config", "kinematics.yaml")
    )
    robot_description_kinematics = {
        "robot_description_kinematics": _robot_description_kinematics_yaml
    }

    # Joint limits
    joint_limits = {
        "robot_description_planning": load_yaml(
            moveit_config_package, path.join("config", "joint_limits.yaml")
        )
    }

    # Servo
    servo_params = {
        "moveit_servo": load_yaml(
            moveit_config_package, path.join("config", "servo.yaml")
        )
    }
    servo_params["moveit_servo"].update({"use_gazebo": use_sim_time})

    # Planning pipeline
    planning_pipeline = {
        "planning_pipelines": ["ompl"],
        "default_planning_pipeline": "ompl",
        "ompl": {
            "planning_plugin": "ompl_interface/OMPLPlanner",
            # TODO: Re-enable `default_planner_request_adapters/AddRuckigTrajectorySmoothing` once its issues are resolved
            "request_adapters": "default_planner_request_adapters/AddTimeOptimalParameterization default_planner_request_adapters/ResolveConstraintFrames default_planner_request_adapters/FixWorkspaceBounds default_planner_request_adapters/FixStartStateBounds default_planner_request_adapters/FixStartStateCollision default_planner_request_adapters/FixStartStatePathConstraints",
            # TODO: Reduce start_state_max_bounds_error once spawning with specific joint configuration is enabled
            # "start_state_max_bounds_error": 0.31416,
            "start_state_max_bounds_error": 0.1,
        },
    }
    _ompl_yaml = load_yaml(
        moveit_config_package, path.join("config", "ompl_planning.yaml")
    )
    planning_pipeline["ompl"].update(_ompl_yaml)

     # Planning scene
    planning_scene_monitor_parameters = {
        "publish_planning_scene": True,
        "publish_geometry_updates": True,
        "publish_state_updates": True,
        "publish_transforms_updates": True,
        "publish_robot_description": True,
        "publish_robot_description_semantic": True, 
    }

    # MoveIt controller manager
    moveit_controller_manager_yaml = load_yaml(
        moveit_config_package, path.join("config", "chessaton_controllers.yaml")
    )
    moveit_controller_manager = {
        "moveit_controller_manager": "moveit_simple_controller_manager/MoveItSimpleControllerManager",
        "moveit_simple_controller_manager": moveit_controller_manager_yaml,
    }

    # Trajectory execution
    trajectory_execution = {
        "allow_trajectory_execution": True,
        "moveit_manage_controllers": True,
        "publish_monitored_planning_scene": True,
        "capabilities": "move_group/ExecuteTaskSolutionCapability",
        "trajectory_execution.allowed_execution_duration_scaling": 1.2,
        "trajectory_execution.allowed_goal_duration_margin": 0.5,
        "trajectory_execution.allowed_start_tolerance": 0.01,
    }

    # Controller parameters
    declared_arguments.append(
        DeclareLaunchArgument(
            "__controller_parameters_basename",
            default_value=["controllers_", ros2_control_command_interface, ".yaml"],
        )
    )
    controller_parameters = PathJoinSubstitution(
        [
            FindPackageShare(description_package),
            "config",
            LaunchConfiguration("__controller_parameters_basename"),
        ]
    )

    # sensors
    octomap_config = {'octomap_frame': 'camera_link_optical', 
                      'octomap_resolution': 0.005,
                      'max_range': 5.0} 
    octomap_updater_config = load_yaml(moveit_config_package, "config/sensor_3d.yaml")

    # List of processes to be executed
    # xacro2sdf
    xacro2sdf = ExecuteProcess(
        cmd=[
            PathJoinSubstitution([FindExecutable(name="ros2")]),
            "run",
            description_package,
            "xacro2sdf.bash",
            ["name:=", name],
            ["prefix:=", prefix],
            ["ros2_control:=", "true"],
            ["ros2_control_plugin:=", "gz"],
            ["ros2_control_command_interface:=", ros2_control_command_interface],
            ["gazebo_preserve_fixed_joint:=", gazebo_preserve_fixed_joint],
        ],
        shell=True,
    )

    processes = [xacro2sdf]

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
        # add 'verbos': 'true' to enable gazebo debug
        launch_arguments={'world': world, 'pause': 'false'}.items(),
        condition=IfCondition(gazebo_sim),
    )

    spawn_entity=Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description',
                   '-entity', 'chessaton'],
        output='screen',
        condition=IfCondition(gazebo_sim),
        )

    robot_state_publisher=Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        arguments=["--ros-args", "--log-level", log_level],
        parameters=[robot_description, {'use_sim_time': use_sim_time}]
    )

    joint_state_broadcaster_spawner=Node(
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

    # ros2_control_node (only for fake controller)
    fake_ros2_controller_node=Node(
        package="controller_manager",
        executable="ros2_control_node",
        output="log",
        arguments=["--ros-args", "--log-level", log_level],
        parameters=[
            robot_description,
            controller_parameters,
            {"use_sim_time": use_sim_time},
        ],
        condition=(
            IfCondition(
                PythonExpression(
                    [
                        "'",
                        ros2_control_plugin,
                        "'",
                        " == ",
                        "'fake'",
                    ]
                )
            )
        ),
    )

    # move_group (with execution)
    run_move_group_node=Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        arguments=["--ros-args", "--log-level", log_level],
        parameters=[
            robot_description,
            robot_description_semantic,
            robot_description_kinematics,
            joint_limits,
            planning_pipeline,
            trajectory_execution,
            planning_scene_monitor_parameters,
            moveit_controller_manager,
            octomap_config,
            octomap_updater_config,
            {"use_sim_time": use_sim_time},
        ],
    )

    # move_servo
    run_move_servo_node=Node(
        package="moveit_servo",
        executable="servo_node_main",
        output="log",
        arguments=["--ros-args", "--log-level", log_level],
        parameters=[
            robot_description,
            robot_description_semantic,
            robot_description_kinematics,
            joint_limits,
            planning_pipeline,
            trajectory_execution,
            planning_scene_monitor_parameters,
            servo_params,
            {"use_sim_time": use_sim_time},
        ],
        condition=IfCondition(enable_servo),
    )

    # rviz2
    rviz_node_full=Node(
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
        parameters=[
            robot_description,
            robot_description_semantic,
            robot_description_kinematics,
            planning_pipeline,
            joint_limits,
            {"use_sim_time": use_sim_time},
        ],
        condition=IfCondition(enable_rviz),
    )

    # chess robot node
    chess_robot_node = Node(
        package="chessaton_chess_manager",
        executable= robot_program,
        output="screen",
        parameters=[
            robot_description,
            robot_description_semantic,
            robot_description_kinematics,
            planning_pipeline,
            joint_limits,
            {'use_sim_time': use_sim_time},
        ],
    )

    # chess engine node
    chess_engine_node = Node(
        package="chessaton_chess_manager",
        executable= "chess_engine",
        output="screen",
        parameters=[
            robot_description,
            robot_description_semantic,
            robot_description_kinematics,
            planning_pipeline,
            joint_limits,
            {'use_sim_time': use_sim_time},
        ],
    )

    return LaunchDescription(declared_arguments+
        [
            # Gazebo nodes:
            gazebo,
            spawn_entity,
            # ROS2_control:
            robot_state_publisher,

            # ROS2 Controllers:
            fake_ros2_controller_node,

            joint_state_broadcaster_spawner,

            RegisterEventHandler(
                OnProcessExit(
                    target_action = joint_state_broadcaster_spawner,
                    on_exit = [
                        joint_trajectory_controller_spawner,
                    ]
                )
            ),
            RegisterEventHandler(
                OnProcessExit(
                    target_action = joint_trajectory_controller_spawner,
                    on_exit = [
                        # left_finger_controller_spawner,
                        chessaton_hand_controller_spawner,
                    ]
                )
            ),
            RegisterEventHandler(
                OnProcessExit(
                    target_action = chessaton_hand_controller_spawner,
                    on_exit = [
                        TimerAction(
                            period=1.0,
                            actions=[
                                rviz_node_full,
                                run_move_group_node
                            ]
                        ),
                    ]
                )
            ),
            TimerAction(
                period=1.0,
                actions=[
                    chess_engine_node,
                ]
            ),
            TimerAction(
                period=2.0,
                actions=[
                    chess_robot_node,
                ]
            ),
        ]
    )


def load_yaml(package_name: str, file_path: str):
    """
    Load yaml configuration based on package name and file path relative to its share.
    """

    package_path = get_package_share_directory(package_name)
    absolute_file_path = path.join(package_path, file_path)
    return parse_yaml(absolute_file_path)


def parse_yaml(absolute_file_path: str):
    """
    Parse yaml from file, given its absolute file path.
    """

    try:
        with open(absolute_file_path, "r") as file:
            return yaml.safe_load(file)
    except EnvironmentError:
        return None


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
        # Gazebo
        DeclareLaunchArgument(
            "gazebo_sim",
            default_value="true",
            description="enable gazebo simulation.",
        ),
        DeclareLaunchArgument(
            "world",
            default_value=path.join(get_package_share_directory('chessaton_chess_manager'), "worlds", "chessboard"),
            description="Name or filepath of world to load.",
        ),
        DeclareLaunchArgument(
            "model",
            default_value="chessaton",
            description="Name or filepath of model to load.",
        ),
        DeclareLaunchArgument(
            "gazebo_preserve_fixed_joint",
            default_value="false",
            description="Flag to preserve fixed joints and prevent lumping when generating SDF for Gazebo.",
        ),
        # Servo
        DeclareLaunchArgument(
            "enable_servo",
            default_value="false",
            description="Flag to enable MoveIt2 Servo for manipulator.",
        ),
        # Miscellaneous
        DeclareLaunchArgument(
            "enable_rviz", default_value="true", description="Flag to enable RViz2."
        ),
        DeclareLaunchArgument(
            "rviz_config",
            default_value=path.join(
                get_package_share_directory("chessaton_control"),
                "rviz",
                "movep_demo.rviz",
            ),
            description="Path to configuration for RViz2.",
        ),
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
        DeclareLaunchArgument(
            "robot_program",
            default_value="chess_robot",
            description="name of the demo program to run.",
        ),
    ]
