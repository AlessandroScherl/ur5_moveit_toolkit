#!/usr/bin/env python3
"""
Minimal pick-and-place skeleton — copy this file to start your own project.

Run against the simulator first:
  T1: ros2 launch ur5_moveit_toolkit ur5_sim.launch.py
  T2: python3 examples/pick_and_place.py --no-gripper

Then on the real robot (driver + MoveIt running, see README):
  python3 examples/pick_and_place.py
"""
import argparse
import math

from ur5_moveit_toolkit import UR5, RobotiqGripper

# Joint poses in radians. Teach your own with:  ros2 run ur5_moveit_toolkit save_joints my_pose.json
HOME = [-0.1156, -1.6593, 1.3059, -1.2197, -1.5684, -1.6848]   # ART-VS home (camera looks at table)
PLACE = [0.4215, -1.0732, 0.4404, -1.0536, -1.5554, -1.1615]   # ART-VS drop pose above container

parser = argparse.ArgumentParser()
parser.add_argument("--no-gripper", action="store_true")
args = parser.parse_args()

gripper = None if args.no_gripper else RobotiqGripper("/dev/ttyUSB0")

with UR5() as robot:
    # Tell MoveIt about the table so it never plans through it.
    # Box centre 2 cm below the base, 1.5 m x 1.5 m.
    robot.add_box("table", size=(1.5, 1.5, 0.02), position=(0.0, 0.0, -0.02))

    # 1) planned joint move (scaled_joint_trajectory_controller)
    robot.move_joints(HOME, speed=0.3)
    if gripper:
        gripper.open()

    # 2) straight-line approach: 8 cm along the tool Z axis
    robot.move_relative(dz=0.08, frame="tool", speed=0.1)
    if gripper:
        gripper.close()

    # 3) lift 10 cm straight up in the base frame
    robot.move_relative(dz=0.10, frame="base", speed=0.1)

    # 4) a short Servo jog (forward_position_controller), e.g. for fine alignment.
    #    use_servo_mode()/use_trajectory_mode() are called automatically.
    robot.jog(angular=(0.0, 0.0, math.radians(10)), duration=1.0)  # 10 deg/s about tool Z

    # 5) back to planning, go to the drop pose
    robot.move_joints(PLACE, speed=0.3)
    if gripper:
        gripper.open()
    robot.move_joints(HOME, speed=0.3)
    print("pick & place finished")
