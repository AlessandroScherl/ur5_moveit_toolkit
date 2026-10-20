"""
UR5 driver bringup (terminal 1).

Wraps ur_robot_driver/ur_control.launch.py and selects the robot description:
  camera_mount:=true  -> urdf/ur5_with_realsense.urdf.xacro (UR5 + D435 bracket)
  camera_mount:=false -> stock ur_robot_driver/urdf/ur.urdf.xacro

Examples:
  ros2 launch ur5_moveit_toolkit ur5_driver.launch.py robot_ip:=10.0.0.30
  ros2 launch ur5_moveit_toolkit ur5_driver.launch.py use_mock_hardware:=true   # no robot needed
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os


def launch_setup(context):
    camera_mount = LaunchConfiguration("camera_mount").perform(context) == "true"
    use_mock = LaunchConfiguration("use_mock_hardware").perform(context) == "true"
    description_file = LaunchConfiguration("description_file").perform(context)

    if not description_file:
        if camera_mount:
            description_file = os.path.join(
                get_package_share_directory("ur5_moveit_toolkit"),
                "urdf", "ur5_with_realsense.urdf.xacro")
        else:
            description_file = os.path.join(
                get_package_share_directory("ur_robot_driver"), "urdf", "ur.urdf.xacro")

    launch_args = {
        name: LaunchConfiguration(name).perform(context)
        for name in ["ur_type", "robot_ip", "headless_mode", "launch_rviz", "use_mock_hardware"]
    }
    launch_args.update({
        "description_file": description_file,
        # The dashboard client needs a real robot to talk to.
        "launch_dashboard_client": "false" if use_mock else "true",
    })
    # (not named kinematics_params_file: an empty value would be inherited by ur_control)
    calibration = LaunchConfiguration("calibration_file").perform(context)
    if calibration:
        launch_args["kinematics_params_file"] = calibration

    ur_control = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory("ur_robot_driver"), "launch", "ur_control.launch.py")),
        launch_arguments=launch_args.items(),
    )
    return [ur_control]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("robot_ip", default_value="10.0.0.30",
                              description="IP address of the UR controller"),
        DeclareLaunchArgument("ur_type", default_value="ur5"),
        DeclareLaunchArgument("headless_mode", default_value="true",
                              description="true: driver sends the URScript itself, no "
                                          "External Control program needed on the pendant"),
        DeclareLaunchArgument("camera_mount", default_value="true",
                              description="Use the UR5 + RealSense mount URDF"),
        DeclareLaunchArgument("description_file", default_value="",
                              description="Override: path to your own URDF/xacro"),
        DeclareLaunchArgument("calibration_file", default_value="",
                              description="Calibration file from ur_calibration (recommended)"),
        DeclareLaunchArgument("use_mock_hardware", default_value="false",
                              description="Fake hardware, for testing without a robot"),
        DeclareLaunchArgument("launch_rviz", default_value="false"),
        OpaqueFunction(function=launch_setup),
    ])
