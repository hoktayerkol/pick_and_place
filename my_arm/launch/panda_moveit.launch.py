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
        # moveit_configs.robot_description
        # moveit_configs.robot_description_semantic
        # moveit_configs.robot_description_kinematics
        # moveit_configs.planning_pipelines
        # moveit_configs.trajectory_execution
        # moveit_configs.planning_scene_monitor
        # moveit_configs.sensors_3d
        # moveit_configs.move_group_capabilities
        # moveit_configs.joint_limits
        # moveit_configs.moveit_cpp
        # moveit_configs.pilz_cartesian_limits
        # # Or to get all the parameters as a dictionary
        # moveit_configs.to_dict()        
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

    # Load  ExecuteTaskSolutionCapability so we can execute found solutions in simulation
    move_group_capabilities = {
        "capabilities": "move_group/ExecuteTaskSolutionCapability"
    }

    # Start the actual move_group node/action server
    run_move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            move_group_capabilities,
        ],
    )

    # RViz
    rviz_config_file = (
        get_package_share_directory("my_arm") + "/config/moveit.rviz"
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
        ],
    )

    # Static TF
    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        output="log",
        arguments=["--frame-id", "world", "--child-frame-id", "panda_link0"],
    )

    # Publish TF
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[
            moveit_config.robot_description,
        ],
    )

    # ros2_control using FakeSystem as hardware
    ros2_controllers_path = os.path.join(
        get_package_share_directory("moveit_resources_panda_moveit_config"),
        "config",
        "ros2_controllers.yaml",
    )
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[moveit_config.to_dict(), ros2_controllers_path],
        output="both",
    )

    # Load controllers
    load_controllers = []
    for controller in [
        "panda_arm_controller",
        "panda_hand_controller",
        "joint_state_broadcaster",
    ]:
        load_controllers += [
            ExecuteProcess(
                cmd=["ros2 run controller_manager spawner {}".format(controller)],
                shell=True,
                output="screen",
            )
        ]

    return LaunchDescription(
        [
            rviz_node,
            static_tf,
            robot_state_publisher,
            run_move_group_node,
            ros2_control_node,
        ]
        + load_controllers
    )