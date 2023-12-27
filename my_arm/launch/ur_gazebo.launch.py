from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    initial_joint_controllers = PathJoinSubstitution(
        [FindPackageShare("my_arm"), "config", "ur_controllers.yaml"]
    )
    
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare("ur_description"), "urdf", "ur.urdf.xacro"]
            ),
            " ",
            "safety_limits:=",
            "true",
            " ",
            "safety_pos_margin:=",
            "0.15",
            " ",
            "safety_k_position:=",
            "20",
            " ",
            "name:=",
            "ur",
            " ",
            "ur_type:=",
            "ur5e", # arm model
            " ",
            "prefix:=",
            '""',   # "Prefix of the joint names, useful for multi-robot setup
            " ",
            "sim_gazebo:=true",
            " ",
            "simulation_controllers:=",
            initial_joint_controllers,  # controllers yaml nin full path
        ]
    )
    robot_description = {"robot_description": robot_description_content}
    
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[{"use_sim_time": True}, robot_description],
    )
    
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )
    
    initial_joint_controller_spawner_started = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_trajectory_controller", "-c", "/controller_manager"],
    )
    
    # Gazebo nodes
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("gazebo_ros"), "/launch", "/gazebo.launch.py"]
        ),
    )
    
    # Spawn robot
    gazebo_spawn_robot = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        name="spawn_ur",
        arguments=["-entity", "ur", "-topic", "robot_description"],
        output="screen",
    )

    
    nodes_to_start = [
        robot_state_publisher_node,
        joint_state_broadcaster_spawner,
        initial_joint_controller_spawner_started,
        gazebo,
        gazebo_spawn_robot,
    ]

    return LaunchDescription(nodes_to_start)