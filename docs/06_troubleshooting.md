# 6. Troubleshooting

Lessons collected while bringing up ART-VS on the UR5 with ROS 2 Jazzy.

## Driver / connection

| Symptom | Fix |
|---|---|
| `ping 10.0.0.30` fails | cable, laptop IP in same subnet (`ip -brief addr`), robot IP on pendant |
| Driver starts, robot does not move, no `Robot connected to reverse interface` | firewall blocks ports 50001–50004; or (Option B) program not playing on the pendant; or wrong Host IP in the URCap |
| Worked, then stopped after a protective stop / E-stop | unlock on the pendant, then `ros2 service call /io_and_status_controller/resend_robot_program std_srvs/srv/Trigger` (headless) or press Play (pendant program) |
| `Could not enable FIFO RT scheduling policy` | harmless warning (see installation 1.4) |
| Two robots/laptops interfere | each group uses its own `export ROS_DOMAIN_ID=<0-101>` |

## MoveIt

| Symptom | Fix |
|---|---|
| `Planning failed` | target unreachable, in collision, or `cartesian=True` path partly impossible. Try a non-Cartesian move, check collision boxes |
| Execution `STATUS_ABORTED` right after servoing | trajectory controller not active → `switch_controller trajectory` (the toolkit does it automatically) |
| Robot moves to a pose rotated by 180° about Z | you used `base_link` instead of `base` |
| Joint values look shuffled | `/joint_states` is alphabetical, not UR order; use `UR5.joints()` |
| Script hangs at a pymoveit2 call | pymoveit2 4.2 can deadlock if the `execute_trajectory` action server is not ready; also don't call its blocking `plan()`/`wait_until_executed()` while the node spins in another executor. `UR5` avoids both |

## MoveIt Servo

| Symptom | Fix |
|---|---|
| Twists published, nothing moves | servo mode not active (`switch_controller servo`), command type not TWIST, or near a singularity (`Very close to a singularity, emergency stop` in the log) → move to a bent home pose |
| Robot stutters / stops every few iterations | commands arrive slower than `incoming_command_timeout` (0.1 s). Publish at 100 Hz from a dedicated thread; for heavy GPU loops use `servo_config:=visual_servoing` (2 s timeout) |
| Python thread can't keep 100 Hz, process at ~100 % CPU while idle | pymoveit2 + rclpy `MultiThreadedExecutor` busy-spins and starves other threads of the GIL. Use a `SingleThreadedExecutor` (as `UR5` does). Measured: 22 Hz → 98 Hz publish loop |
| servo_node crashes at start with unknown plugin | `NoSmoothingPlugin` doesn't exist in Jazzy; use `ButterworthFilterPlugin` |
| servo_node segfaults at start | it was started before `/robot_description` existed; `ur5_moveit.launch.py` waits for it |
| Achieved velocity smaller than commanded | Servo integrates from the measured state; with latency (strongly on mock hardware, ~25 %) it under-shoots. Fine for closed-loop control; use `move_relative` for exact offsets |

## Python / environment

| Symptom | Fix |
|---|---|
| `ModuleNotFoundError: ur5_moveit_toolkit` | `source ~/ur5_ws/install/setup.bash` |
| `ModuleNotFoundError: rclpy` in conda | create the env with python 3.12 and `source /opt/ros/jazzy/setup.bash` after `conda activate` |
| `cv_bridge` / numpy errors | `pip install "numpy<2"` |
| Gripper: `Permission denied: /dev/ttyUSB0` | `sudo usermod -aG dialout $USER`, log out/in |
| Sudden CUDA slowdown / erratic behaviour after an update | unattended-upgrades replaced the NVIDIA driver → reboot. Prevent: `sudo apt-mark hold nvidia-driver-535` (your version) |

## Useful debugging commands

```bash
ros2 control list_controllers
ros2 topic echo /servo_node/status
ros2 topic hz /servo_node/delta_twist_cmds
ros2 run tf2_ros tf2_echo base tool0
ros2 run tf2_tools view_frames          # PDF of the TF tree
ros2 service call /dashboard_client/get_robot_mode ur_dashboard_msgs/srv/GetRobotMode
```
