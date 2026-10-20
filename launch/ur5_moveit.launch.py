"""
MoveIt 2 + MoveIt Servo for the UR5 (terminal 2). Start ur5_driver.launch.py first.

Includes the stock ur_moveit_config/ur_moveit.launch.py (move_group + RViz) with its
built-in servo disabled, then starts our own servo_node with a selectable config:
  servo_config:=default          -> config/servo_default.yaml (stock UR, 0.1 s watchdog)
  servo_config:=visual_servoing  -> config/servo_visual_servoing.yaml (ART-VS tuned)
  servo_config:=/abs/path.yaml   -> your own file

Examples:
  ros2 launch ur5_moveit_toolkit ur5_moveit.launch.py
  ros2 launch ur5_moveit_toolkit ur5_moveit.launch.py servo_config:=visual_servoing launch_rviz:=false
"""
import os
from pathlib import Path

import yaml
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction,
                            RegisterEventHandler)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder
from ament_index_python.packages import get_package_share_directory


def launch_setup(context):
    ur_type = LaunchConfiguration("ur_type").perform(context)
    servo_config = LaunchConfiguration("servo_config").perform(context)
    launch_servo = LaunchConfiguration("launch_servo").perform(context) == "true"
    launch_rviz = LaunchConfiguration("launch_rviz").perform(context)

    if not servo_config.endswith(".yaml"):
        servo_config = os.path.join(
            get_package_share_directory("ur5_moveit_toolkit"), "config",
            f"servo_{servo_config}.yaml")
    with open(servo_config) as f:
        servo_params = {"moveit_servo": yaml.safe_load(f)}

    # NOTE: included launch files share launch configurations with this file, so all of
    # our own arguments are resolved above, before the include sets launch_servo:=false.
    # (Do not wrap it in a scoped GroupAction: ur_moveit starts move_group from an event
    # handler that needs its launch configurations after the group has ended.)
    ur_moveit = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory("ur_moveit_config"), "launch", "ur_moveit.launch.py")),
        launch_arguments={
            "ur_type": ur_type,
            "launch_servo": "false",  # replaced by the servo_node below
            "launch_rviz": launch_rviz,
        }.items(),
    )
    if not launch_servo:
        return [ur_moveit]

    moveit_config = (
        MoveItConfigsBuilder(robot_name="ur", package_name="ur_moveit_config")
        .robot_description_semantic(Path("srdf") / "ur.srdf.xacro", {"name": ur_type})
        .to_moveit_configs()
    )
    servo_node = Node(
        package="moveit_servo",
        executable="servo_node",
        output="screen",
        parameters=[moveit_config.to_dict(), servo_params],
    )
    # Servo crashes if started before the driver publishes /robot_description
    wait_robot_description = Node(
        package="ur_robot_driver", executable="wait_for_robot_description", output="screen")
    start_servo = RegisterEventHandler(
        OnProcessExit(target_action=wait_robot_description, on_exit=[servo_node]))
    return [ur_moveit, wait_robot_description, start_servo]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("ur_type", default_value="ur5"),
        DeclareLaunchArgument("launch_rviz", default_value="true"),
        DeclareLaunchArgument("launch_servo", default_value="true"),
        DeclareLaunchArgument("servo_config", default_value="default",
                              description="'default', 'visual_servoing' or a .yaml path"),
        OpaqueFunction(function=launch_setup),
    ])
