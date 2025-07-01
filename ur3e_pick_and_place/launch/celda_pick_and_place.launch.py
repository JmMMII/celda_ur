from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():
    moveit_config = (
        MoveItConfigsBuilder(robot_name="robot_celda", package_name="celda_moveit_config").planning_pipelines().to_moveit_configs()
    )

    return LaunchDescription([
        Node(
            package="ur3e_pick_and_place",
            executable="celda_pick_and_place",
            output="screen",
            parameters=[
                moveit_config.robot_description,
                moveit_config.robot_description_semantic,
                moveit_config.robot_description_kinematics,
                # moveit_config.robot_description_planning,
                moveit_config.trajectory_execution,
                moveit_config.joint_limits,
                moveit_config.planning_pipelines
            ]
        )
    ])
