import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():

    robot_name = "moveit_resources_panda"
    roboot_description_pkg = "moveit_resources_panda_description"
    roboot_config_pkg = "moveit_resources_panda_moveit_config"

    roboot_description_path = get_package_share_directory(roboot_description_pkg)
    roboot_config_path = get_package_share_directory(roboot_config_pkg)

    roboot_description_file = os.path.join(roboot_config_path, "config", "panda.urdf.xacro")
    robot_description_semantic_file = os.path.join(roboot_config_path, "config", "panda.srdf")
    roboot_config_file = os.path.join(roboot_config_path, "config", "gripper_moveit_controllers.yaml")

    rviz_config_file = os.path.join(get_package_share_directory("pick_and_place"), "launch", "moveit.rviz")

    
    # planning_context
    moveit_config = (
        MoveItConfigsBuilder(robot_name)
        .robot_description(file_path=roboot_description_file)
        .robot_description_semantic(file_path=robot_description_semantic_file)
        .trajectory_execution(file_path=roboot_config_file)
        .planning_pipelines(pipelines=["ompl", "chomp", "pilz_industrial_motion_planner"])
        .to_moveit_configs()
        # moveit_configs.package_path      
    )
    
    '''
    # planning_context
    moveit_config = (
        MoveItConfigsBuilder("moveit_resources_panda")
        .robot_description(file_path="config/panda.urdf.xacro")
        .trajectory_execution(file_path="config/gripper_moveit_controllers.yaml")
        .planning_pipelines(
            pipelines=["ompl", "chomp", "pilz_industrial_motion_planner"]
        )
        .to_moveit_configs()
    )
    '''    
   
        
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[{"use_sim_time": True}, moveit_config.robot_description],
    )

    joint_state_publisher = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        name="joint_state_publisher",
    )
    
    from launch.actions import IncludeLaunchDescription
    from launch.launch_description_sources import PythonLaunchDescriptionSource
    from launch_ros.substitutions import FindPackageShare

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
        arguments=["-entity", "panda", "-topic", "robot_description"],
        output="screen",
    )

    ld = LaunchDescription()
    #ld.add_action(robot_state_publisher_node)
    #ld.add_action(joint_state_publisher)
    ld.add_action(gazebo)
    ld.add_action(gazebo_spawn_robot)

    return ld