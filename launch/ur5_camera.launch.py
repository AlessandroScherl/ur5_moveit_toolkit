"""
Intel RealSense D435/D435i on the UR5 wrist (optional, terminal 3).

Same camera profile as the ART-VS setup. The URDF publishes tool0 -> camera_link;
the RealSense driver publishes camera_link -> camera_color_optical_frame, so the full
TF chain base -> tool0 -> camera_color_optical_frame is available once both run.

Note: in ROS 2 the topics are double-namespaced: /camera/camera/color/image_raw

Example:
  ros2 launch ur5_moveit_toolkit ur5_camera.launch.py fps:=15
"""
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    fps = LaunchConfiguration("fps")
    rs_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory("realsense2_camera"), "launch", "rs_launch.py")),
        launch_arguments={
            "enable_color": "true",
            "enable_depth": "true",
            "align_depth.enable": "true",
            # camera_link -> optical frames (tool0 -> camera_link comes from the URDF)
            "publish_tf": "true",
            "rgb_camera.color_profile": PythonExpression(["'1920x1080x' + '", fps, "'"]),
            "depth_module.depth_profile": PythonExpression(["'1280x720x' + '", fps, "'"]),
        }.items(),
    )
    return LaunchDescription([
        DeclareLaunchArgument("fps", default_value="6", description="6, 15 or 30"),
        rs_launch,
    ])
