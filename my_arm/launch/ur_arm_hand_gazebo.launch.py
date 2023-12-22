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
        [FindPackageShare("ur_arm_hand_moveit_config"), "config", "ros2_controllers.yaml"]
    )
    
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare("xacro2moveit"), "script", "ur_arm_hand_gazebo.urdf.xacro"]
            ),
            " ",
            "name:=",
            "ur_arm_hand",
            " ",
            "ur_type:=",
            "ur5e", # arm model
            " ",
            "sim_gazebo:=true",
            " ",
            "safety_limits:=true",
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
        # arguments=["joint_trajectory_controller", "-c", "/controller_manager"],
        arguments=["arm_group_controller", "-c", "/controller_manager"],
    )
    initial_joint_controller_spawner_started2 = Node(
        package="controller_manager",
        executable="spawner",
        # arguments=["joint_trajectory_controller", "-c", "/controller_manager"],
        arguments=["hand_group_controller", "-c", "/controller_manager"],
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
        initial_joint_controller_spawner_started2,
        gazebo,
        gazebo_spawn_robot,
    ]

    return LaunchDescription(nodes_to_start)