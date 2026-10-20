# ur5_moveit_toolkit

Everything you need to move a **Universal Robots UR5** with **ROS 2 Jazzy + MoveIt 2**:
bringup launch files, a tuned MoveIt Servo setup, controller switching, command-line tools
and a small Python API. It was extracted from the ART-VS (ViT-VS) real-robot pipeline, where it was used for
visual servoing and pick-and-place.

```python
from ur5_moveit_toolkit import UR5

with UR5() as robot:
    robot.move_joints([-0.12, -1.66, 1.31, -1.22, -1.57, -1.68])  # planned joint move
    robot.move_relative(dz=-0.05)                                  # 5 cm straight down
    robot.jog(linear=(0, 0, 0.02), duration=1.0)                   # MoveIt Servo velocity
```

> **Safety first.** Keep the teach pendant's emergency stop within reach whenever the
> robot is powered. Try every new script in the simulator (`ur5_sim.launch.py`) first,
> then on the real robot with the speed slider on the pendant at 10–30 %.

---

## 1. Install (once per laptop)

Ubuntu 24.04 + ROS 2 Jazzy ([install guide](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html)), then:

```bash
mkdir -p ~/ur5_ws/src && cd ~/ur5_ws/src
git clone <this-repo-url> ur5_moveit_toolkit
./ur5_moveit_toolkit/install_dependencies.sh      # apt packages (asks for sudo)
cd ~/ur5_ws && colcon build --symlink-install
echo "source ~/ur5_ws/install/setup.bash" >> ~/.bashrc && source ~/.bashrc
```

Details, conda caveats and the network setup: [docs/01_installation.md](docs/01_installation.md),
[docs/02_robot_and_network_setup.md](docs/02_robot_and_network_setup.md).

## 2. Try it without a robot (simulator)

```bash
ros2 launch ur5_moveit_toolkit ur5_sim.launch.py          # fake hardware + MoveIt + Servo + RViz
# second terminal:
ros2 run ur5_moveit_toolkit status
ros2 run ur5_moveit_toolkit move_joints --deg 0 -90 90 -90 -90 0
ros2 run ur5_moveit_toolkit move_tcp --dz -0.05
python3 ~/ur5_ws/src/ur5_moveit_toolkit/examples/pick_and_place.py --no-gripper
```

## 3. Run the real UR5

On the robot: powered on, brakes released, **Remote/External Control possible** (see
[docs/02](docs/02_robot_and_network_setup.md)). Laptop cable in the robot network. Then:

| Terminal | Command |
|---|---|
| T1 – driver | `ros2 launch ur5_moveit_toolkit ur5_driver.launch.py robot_ip:=10.0.0.30` |
| T2 – MoveIt + Servo | `ros2 launch ur5_moveit_toolkit ur5_moveit.launch.py` |
| T3 – camera (optional) | `ros2 launch ur5_moveit_toolkit ur5_camera.launch.py fps:=15` |
| T4 – your code | `ros2 run ur5_moveit_toolkit status` / your script |

Wait for `You can start planning now!` in T2 before you move anything.

Useful launch arguments:

| Launch file | Argument | Default | Meaning |
|---|---|---|---|
| `ur5_driver` | `robot_ip` | `10.0.0.30` | IP of the UR controller |
| | `headless_mode` | `true` | driver sends the program itself (no pendant program needed) |
| | `camera_mount` | `true` | URDF with the RealSense bracket on `tool0`; `false` = bare UR5 |
| | `description_file` | – | your own URDF/xacro instead |
| | `calibration_file` | – | factory calibration from `ur_calibration` (more accurate) |
| | `use_mock_hardware` | `false` | fake robot |
| `ur5_moveit` | `servo_config` | `default` | `default`, `visual_servoing` or a `.yaml` path |
| | `launch_rviz` | `true` | RViz with the MotionPlanning panel |

## 4. Two ways to move the robot → two controllers

| Mode | Active controller | Used by | How you command it |
|---|---|---|---|
| **trajectory** | `scaled_joint_trajectory_controller` | MoveIt planning (`move_joints`, `move_tcp`, RViz *Plan & Execute*) | goal pose / joint angles |
| **servo** | `forward_position_controller` | MoveIt Servo (`jog`, visual servoing, teleop) | velocity (twist) at ≥ 50 Hz |

Only one can be active. Switch with:

```bash
ros2 run ur5_moveit_toolkit switch_controller servo        # or: trajectory | status
```

The Python API and the CLI tools switch automatically. Full explanation and the raw ROS
commands: [docs/03_controllers.md](docs/03_controllers.md).

## 5. Command-line tools

| Command | What it does |
|---|---|
| `ros2 run ur5_moveit_toolkit status` | joint angles, `tool0` pose, active controllers |
| `ros2 run ur5_moveit_toolkit save_joints home.json` | store the current joints (teach by freedrive) |
| `ros2 run ur5_moveit_toolkit move_joints home.json` | planned joint move (`--deg …`, `--rad …`, `--speed 0.2`) |
| `ros2 run ur5_moveit_toolkit move_tcp --dz -0.05` | straight-line relative move (`--frame tool`, `--drz 0.5`) |
| `ros2 run ur5_moveit_toolkit jog --vz 0.02 --time 1` | Servo velocity move (`--frame base`, `--back`) |
| `ros2 run ur5_moveit_toolkit switch_controller servo` | switch controller (`trajectory`, `status`) |
| `ros2 run ur5_moveit_toolkit gripper close` | Robotiq 2F over USB/RS-485 (`open`, `--port`) |

## 6. Python API

See [docs/04_moving_the_robot.md](docs/04_moving_the_robot.md) and
[`examples/pick_and_place.py`](examples/pick_and_place.py).

| Method | |
|---|---|
| `UR5()` / `with UR5() as robot:` | connect (waits for `/joint_states`) |
| `joints()` | 6 joint angles [rad], UR order |
| `tcp_pose()` | `[x, y, z, rx, ry, rz]` of `tool0` in `base` (same format as the pendant / ur_rtde) |
| `move_joints(q, speed=0.2)` | planned joint move → `True/False` |
| `move_pose(pose, cartesian=False)` | planned move to a `tool0` pose |
| `move_relative(dx, dy, dz, drx, dry, drz, frame="base"/"tool")` | straight-line offset |
| `jog(linear, angular, duration)` / `send_twist(linear, angular)` | MoveIt Servo velocity commands |
| `use_trajectory_mode()` / `use_servo_mode()` | controller switching |
| `add_box(name, size, position)` / `remove_object(name)` | collision objects (table, walls) |
| `RobotiqGripper("/dev/ttyUSB0").open()/close()` | gripper |

## 7. Repository layout

```
launch/   ur5_driver · ur5_moveit · ur5_camera · ur5_sim  (.launch.py)
config/   servo_default.yaml (stock UR) · servo_visual_servoing.yaml (ART-VS tuned)
urdf/     ur5_with_realsense.urdf.xacro + meshes (camera bracket, D435)
ur5_moveit_toolkit/   Python API: robot.py · controllers.py · gripper.py
scripts/  command-line tools (installed for `ros2 run`)
examples/ pick_and_place.py — copy this to start a project
poses/    saved joint poses (ART-VS home)
docs/     installation · robot/network · controllers · moving · custom tool · troubleshooting
```

## 8. More docs

1. [Installation](docs/01_installation.md)
2. [Robot & network setup](docs/02_robot_and_network_setup.md)
3. [Controllers & switching](docs/03_controllers.md)
4. [Moving the robot (CLI, Python, raw ROS, RViz)](docs/04_moving_the_robot.md)
5. [Custom tool / camera / URDF](docs/05_custom_tool_or_camera.md)
6. [Troubleshooting](docs/06_troubleshooting.md)
