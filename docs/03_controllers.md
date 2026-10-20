# 3. Controllers & switching

## 3.1 Why there are several controllers

`ur_robot_driver` runs **ros2_control**. The controller manager loads many controllers,
but only **one motion controller** can own the joints at a time. The two you need:

| Mode name (toolkit) | Controller | Input | Who uses it |
|---|---|---|---|
| `trajectory` *(active at startup)* | `scaled_joint_trajectory_controller` | complete joint trajectories (action `/scaled_joint_trajectory_controller/follow_joint_trajectory`) | MoveIt `move_group`: `move_joints`, `move_tcp`, RViz *Plan & Execute* |
| `servo` | `forward_position_controller` | a stream of joint positions on `/forward_position_controller/commands` | MoveIt Servo (`servo_node`), which turns **twists** into joint positions |

```
                       ┌────────────────────────────┐
 move_joints / RViz ──►│ move_group (MoveIt)        │──► scaled_joint_trajectory_controller ─┐
                       └────────────────────────────┘                                        ├─► UR5
 jog / VS / teleop ──► /servo_node/delta_twist_cmds ──► servo_node ──► forward_position_controller ─┘
                       (TwistStamped, ≥ 50 Hz)
```

Typical symptoms when the wrong controller is active:

* MoveIt plans fine but execution fails (`STATUS_ABORTED`, "controller not active")
  → you are in **servo** mode, switch to `trajectory`.
* You publish twists and nothing moves → you are in **trajectory** mode, or Servo's
  command type is not `TWIST`.

## 3.2 Switching

### With the toolkit (recommended)

```bash
ros2 run ur5_moveit_toolkit switch_controller servo
ros2 run ur5_moveit_toolkit switch_controller trajectory
ros2 run ur5_moveit_toolkit switch_controller status
```

```python
robot.use_servo_mode()       # also sets Servo's command type to TWIST
robot.use_trajectory_mode()
```

`UR5.move_*()` and `UR5.jog()` call these automatically.

### Raw ROS 2 commands (what the toolkit does under the hood)

```bash
# list controllers and their state
ros2 control list_controllers

# trajectory -> servo
ros2 control switch_controllers \
    --deactivate scaled_joint_trajectory_controller \
    --activate forward_position_controller
ros2 service call /servo_node/switch_command_type \
    moveit_msgs/srv/ServoCommandType "{command_type: 1}"    # 0=JOINT_JOG 1=TWIST 2=POSE

# servo -> trajectory
ros2 control switch_controllers \
    --deactivate forward_position_controller \
    --activate scaled_joint_trajectory_controller
```

The same switch as a service call (e.g. from C++ or another language):

```bash
ros2 service call /controller_manager/switch_controller controller_manager_msgs/srv/SwitchController \
  "{activate_controllers: [forward_position_controller],
    deactivate_controllers: [scaled_joint_trajectory_controller],
    strictness: 2, activate_asap: true}"
```

> ROS 1 used `twist_controller` / `scaled_pos_joint_traj_controller` and
> `rosservice call /controller_manager/switch_controller`. In ROS 2 the names above are
> the ones to use.

## 3.3 Other controllers (advanced)

All loaded but inactive by default (see `ros2 control list_controllers`):

| Controller | Use |
|---|---|
| `joint_trajectory_controller` | like the scaled one but ignores the speed slider |
| `forward_velocity_controller` | raw joint velocities on `/forward_velocity_controller/commands` |
| `passthrough_trajectory_controller` | lets the robot controller interpolate the trajectory |
| `freedrive_mode_controller` | hand-guiding from ROS |
| `force_mode_controller` | UR force mode |

The toolkit's switcher deactivates any of these when you switch to `trajectory`/`servo`.

## 3.4 MoveIt Servo configuration

`ur5_moveit.launch.py servo_config:=...` picks the Servo parameters:

| File | `incoming_command_timeout` | Notes |
|---|---|---|
| `config/servo_default.yaml` | 0.1 s | stock `ur_moveit_config` values. Robot stops 0.1 s after your commands stop. **Use this for teaching / teleop.** |
| `config/servo_visual_servoing.yaml` | 2.0 s | ART-VS settings: tolerates up to 2 s gaps from heavy GPU work in the control loop, no halt messages, tighter collision margins. **The robot keeps moving for up to 2 s if your program hangs.** |

Important parameters:

* `command_in_type: speed_units`: twists in m/s and rad/s.
* `robot_link_command_frame: tool0`: default frame if your twist has an empty `frame_id`.
* `lower/hard_stop_singularity_threshold`: Servo slows down or stops near singularities
  (stretched arm, wrist 2 ≈ 0). Move to a bent "home" pose before servoing.
* `smoothing_filter_plugin_name`: Jazzy only ships `ButterworthFilterPlugin`,
  `AccelerationLimitedPlugin` (needs `update_period`) and `RuckigFilterPlugin`.
  `NoSmoothingPlugin` from older tutorials **does not exist** and crashes servo_node.
