# 4. Moving the robot

Prerequisite: driver + MoveIt running (real robot or `ur5_sim.launch.py`).

## 4.1 Frames and units

| Frame | Meaning |
|---|---|
| `base` | robot base, **same as the pendant's "Base" coordinates**. Used by the toolkit. |
| `base_link` | rotated 180° about Z relative to `base` (ROS convention). MoveIt's planning frame. Don't mix them up. |
| `tool0` | tool flange, no TCP offset. |
| `camera_link`, `camera_color_optical_frame` | RealSense (with `camera_mount:=true`). Optical frame: Z forward, X right, Y down. |

Poses are `[x, y, z, rx, ry, rz]`: metres plus a rotation vector in radians, the same as the
pendant and `ur_rtde` (`getActualTCPPose`). Joint order is the UR order: base, shoulder,
elbow, wrist 1, wrist 2, wrist 3. Note that `/joint_states` itself is **not** in that
order; `UR5.joints()` reorders it for you.

## 4.2 Teaching poses

1. Pendant: *Move* tab, or hold the freedrive button on the back of the pendant and move the arm by hand.
2. Save: `ros2 run ur5_moveit_toolkit save_joints poses/pick_above.json`
3. Go there later: `ros2 run ur5_moveit_toolkit move_joints poses/pick_above.json`

## 4.3 Command-line tools

```bash
ros2 run ur5_moveit_toolkit status
ros2 run ur5_moveit_toolkit move_joints poses/home_artvs.json --speed 0.3
ros2 run ur5_moveit_toolkit move_joints --deg 0 -90 90 -90 -90 0
ros2 run ur5_moveit_toolkit move_tcp --dz -0.05                 # base frame, straight line
ros2 run ur5_moveit_toolkit move_tcp --dz 0.08 --frame tool     # along the tool axis
ros2 run ur5_moveit_toolkit move_tcp --drz 0.3 --frame tool     # rotate about tool Z
ros2 run ur5_moveit_toolkit jog --vx 0.02 --frame base --time 2 # Servo, 2 cm/s for 2 s
```

`--speed` is MoveIt's velocity scaling (0–1). The pendant speed slider scales on top of
that.

## 4.4 Python API

```python
from ur5_moveit_toolkit import UR5, RobotiqGripper

with UR5() as robot:                      # connects, waits for /joint_states
    print(robot.joints(), robot.tcp_pose())

    robot.add_box("table", size=(1.5, 1.5, 0.02), position=(0, 0, -0.02))

    ok = robot.move_joints(HOME, speed=0.3)            # planned, collision-aware
    ok = robot.move_pose([0.4, -0.1, 0.3, 0, 3.14, 0]) # any path to a tool0 pose
    ok = robot.move_pose(target, cartesian=True)       # straight line, fails if impossible
    ok = robot.move_relative(dz=0.08, frame="tool")    # approach along tool Z
    if not ok:
        ...                                            # planning or execution failed
```

All `move_*` methods block until the motion is finished and return `True`/`False`. They
switch to the trajectory controller themselves.

### Servo: velocity control from your own loop

For closed-loop control (visual servoing, tracking, teleoperation) send a twist every
cycle:

```python
import time
robot.use_servo_mode()
while not done:
    v, w = my_controller()                    # m/s, rad/s
    robot.send_twist(v, w, frame="tool0")     # or "base", or a camera frame in the TF tree
    time.sleep(0.01)                          # >= 50 Hz, else Servo's watchdog stops the robot
robot.send_twist()                            # zero = stop
robot.use_trajectory_mode()                   # before the next move_*()
```

* Servo commands are **velocities**. They are not exact displacements, and the achieved
  speed can lag the commanded one (strongly on mock hardware). Use `move_relative` for
  precise offsets.
* If one loop iteration can take longer than `incoming_command_timeout` (e.g. a neural
  network on the GPU), publish the last twist from a separate thread at 100 Hz (as ART-VS
  does) and/or use `servo_config:=visual_servoing`.
* Keep heavy work out of the ROS callback thread. Also do **not** combine pymoveit2 with
  rclpy's `MultiThreadedExecutor`: it busy-spins at ~100 % CPU and starves your other
  threads (see troubleshooting).

## 4.5 Raw ROS 2 (no toolkit)

```bash
# Servo twist in tool0 frame, 1 cm/s along Z (Ctrl+C to stop)
ros2 run ur5_moveit_toolkit switch_controller servo
ros2 topic pub --rate 100 /servo_node/delta_twist_cmds geometry_msgs/msg/TwistStamped \
  "{header: {frame_id: 'tool0'}, twist: {linear: {x: 0.0, y: 0.0, z: 0.01}}}"

# joint trajectory directly to the controller (no MoveIt, no collision checking!)
ros2 run ur5_moveit_toolkit switch_controller trajectory
ros2 action send_goal /scaled_joint_trajectory_controller/follow_joint_trajectory \
  control_msgs/action/FollowJointTrajectory "{trajectory: {joint_names: [shoulder_pan_joint,
  shoulder_lift_joint, elbow_joint, wrist_1_joint, wrist_2_joint, wrist_3_joint],
  points: [{positions: [0.0, -1.57, 1.57, -1.57, -1.57, 0.0], time_from_start: {sec: 6}}]}}"
```

## 4.6 RViz

`ur5_moveit.launch.py` opens RViz with the *MotionPlanning* panel. Drag the interactive
marker, then press *Plan* → check the preview → *Execute*. Under *Planning Group*:
`ur_manipulator`.

## 4.7 Planning tips

* Always add the table (and walls, camera tripods...) as collision boxes. MoveIt otherwise
  happily plans through them.
* `cartesian=True` (straight line) fails if less than 95 % of the path is feasible.
  Move closer, or plan non-Cartesian first.
* Large rotations about the camera axis make `tool0` sweep a big arc (the camera is
  offset from the flange) and often fail to plan near joint limits. Check the return
  value and handle failure.
* Avoid singular poses for Servo: stretched arm, wrist 2 ≈ 0, wrist centre above the base.
