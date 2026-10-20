"""
Everything in one terminal with FAKE hardware: driver (mock) + MoveIt + Servo + RViz.
Use this to learn and to test your scripts before touching the real robot.

  ros2 launch ur5_moveit_toolkit ur5_sim.launch.py
"""
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    launch_dir = os.path.join(get_package_share_directory("ur5_moveit_toolkit"), "launch")
    driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir, "ur5_driver.launch.py")),
        launch_arguments={
            "use_mock_hardware": "true",
            "robot_ip": "0.0.0.0",
            "camera_mount": LaunchConfiguration("camera_mount"),
        }.items(),
    )
    moveit = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir, "ur5_moveit.launch.py")),
        launch_arguments={
            "launch_rviz": LaunchConfiguration("launch_rviz"),
            "servo_config": LaunchConfiguration("servo_config"),
        }.items(),
    )
    return LaunchDescription([
        DeclareLaunchArgument("camera_mount", default_value="true"),
        DeclareLaunchArgument("launch_rviz", default_value="true"),
        DeclareLaunchArgument("servo_config", default_value="default"),
        driver,
        moveit,  # waits for /robot_description by itself
    ])
