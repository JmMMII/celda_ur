# from launch import LaunchDescription
# from launch_ros.actions import Node
# from moveit_configs_utils import MoveItConfigsBuilder

# def generate_launch_description():
#     moveit_config = (
#         MoveItConfigsBuilder(robot_name="ur_onrobot", package_name="ur_onrobot_moveit_config").planning_pipelines().to_moveit_configs()
#     )

#     return LaunchDescription([
#         Node(
#             package="ur3e_pick_and_place",
#             executable="ur3e_pick_and_place",
#             output="screen",
#             parameters=[
#                 moveit_config.robot_description,
#                 moveit_config.robot_description_semantic,
#                 moveit_config.robot_description_kinematics,
#                 # moveit_config.robot_description_planning,
#                 moveit_config.trajectory_execution,
#                 moveit_config.joint_limits,
#                 moveit_config.planning_pipelines
#             ]
#         )
#     ])

import launch
import os
import sys

from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution, Command, FindExecutable
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Initialize Arguments
    ur_type = "ur3e"
    onrobot_type = "rg2"
    safety_limits = "true"
    safety_pos_margin = "0.15"
    safety_k_position = "20"

    # General arguments
    ur_description_package = "ur_description"
    description_file = "ur_onrobot.urdf.xacro"
    # _publish_robot_description_semantic = "True"
    moveit_config_package = "ur_onrobot_moveit_config"
    # moveit_joint_limits_file = "joint_limits.yaml"
    moveit_config_file = "ur_onrobot.srdf.xacro"
    # warehouse_sqlite_path = os.path.expanduser("~/.ros/warehouse_ros.sqlite")
    prefix = '""'
    # use_sim_time = "false"
    # launch_rviz = "false"
    # launch_servo = "false"

    joint_limit_params = PathJoinSubstitution(
        [FindPackageShare(ur_description_package), "config", ur_type, "joint_limits.yaml"]
    )
    kinematics_params = PathJoinSubstitution(
        [FindPackageShare(ur_description_package), "config", ur_type, "default_kinematics.yaml"]
    )
    physical_params = PathJoinSubstitution(
        [FindPackageShare(ur_description_package), "config", ur_type, "physical_parameters.yaml"]
    )
    visual_params = PathJoinSubstitution(
        [FindPackageShare(ur_description_package), "config", ur_type, "visual_parameters.yaml"]
    )

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([FindPackageShare("ur_onrobot_description"), "urdf", description_file]),
            " ",
            "robot_ip:=xxx.yyy.zzz.www",
            " ",
            "joint_limit_params:=",
            joint_limit_params,
            " ",
            "kinematics_params:=",
            kinematics_params,
            " ",
            "physical_params:=",
            physical_params,
            " ",
            "visual_params:=",
            visual_params,
            " ",
            "safety_limits:=",
            safety_limits,
            " ",
            "safety_pos_margin:=",
            safety_pos_margin,
            " ",
            "safety_k_position:=",
            safety_k_position,
            " ",
            "name:=",
            "ur_onrobot",
            " ",
            "ur_type:=",
            ur_type,
            " ",
            "onrobot_type:=",
            onrobot_type,
            " ",
            "script_filename:=ros_control.urscript",
            " ",
            "input_recipe_filename:=rtde_input_recipe.txt",
            " ",
            "output_recipe_filename:=rtde_output_recipe.txt",
            " ",
            "prefix:=",
            prefix,
            " ",
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    # MoveIt Configuration
    robot_description_semantic_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare(moveit_config_package), "srdf", moveit_config_file]
            ),
            " ",
            "name:=",
            # Also ur_type parameter could be used but then the planning group names in yaml
            # configs has to be updated!
            "ur_onrobot",
            " ",
            "prefix:=",
            prefix,
            " ",
        ]
    )
    robot_description_semantic = {"robot_description_semantic": robot_description_semantic_content}

    # generate_common_hybrid_launch_description() returns a list of nodes to launch
    demo_node = Node(
        package="ur3e_pick_and_place",
        executable="ur3e_pick_and_place",
        name="ur3e_pick_and_place",
        output="screen",
        parameters=[
            robot_description,
            robot_description_semantic,
        ],
    )

    return launch.LaunchDescription([demo_node])