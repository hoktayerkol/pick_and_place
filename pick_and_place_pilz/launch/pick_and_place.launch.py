from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # moveit_config = MoveItConfigsBuilder("moveit_resources_panda").to_moveit_configs()
    moveit_config = MoveItConfigsBuilder("ur_arm_hand").to_moveit_configs()

    config = os.path.join(
        get_package_share_directory('pick_and_place_pilz'),
        'params',
        'params.yaml'
    )
    # MoveGroupInterface demo executable
    move_group_demo = Node(
        name="ppp_node",
        package="pick_and_place_pilz",
        executable="rb_node",
        output="screen",
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            {"use_sim_time": True}
            # config
        ],
    )

    return LaunchDescription([move_group_demo])