"""
ros2_control controller switching for the UR driver.

The UR driver can only have ONE motion controller active at a time:

  scaled_joint_trajectory_controller  <- MoveIt planning/execution ("trajectory" mode)
  forward_position_controller         <- MoveIt Servo, real-time twist/jog ("servo" mode)

Equivalent CLI:
  ros2 control switch_controllers --deactivate scaled_joint_trajectory_controller \
                                  --activate forward_position_controller
"""
import time

from controller_manager_msgs.srv import ListControllers, SwitchController
from moveit_msgs.srv import ServoCommandType

TRAJECTORY_CONTROLLER = "scaled_joint_trajectory_controller"
SERVO_CONTROLLER = "forward_position_controller"

MODES = {
    "trajectory": TRAJECTORY_CONTROLLER,
    "servo": SERVO_CONTROLLER,
}

# Controllers that claim the joint command interfaces — at most one may be active.
MOTION_CONTROLLERS = [
    "scaled_joint_trajectory_controller",
    "joint_trajectory_controller",
    "forward_position_controller",
    "forward_velocity_controller",
    "forward_effort_controller",
    "passthrough_trajectory_controller",
    "freedrive_mode_controller",
]


def wait_for_future(future, timeout):
    """Wait for a future that is being completed by an executor spinning in another thread."""
    deadline = time.monotonic() + timeout
    while not future.done():
        if time.monotonic() > deadline:
            return None
        time.sleep(0.01)
    return future.result()


class ControllerSwitcher:
    """Query and switch UR motion controllers. The node must be spun by an executor."""

    def __init__(self, node, callback_group=None, controller_manager="/controller_manager",
                 servo_node="/servo_node"):
        self._node = node
        self._list = node.create_client(
            ListControllers, f"{controller_manager}/list_controllers", callback_group=callback_group)
        self._switch = node.create_client(
            SwitchController, f"{controller_manager}/switch_controller", callback_group=callback_group)
        self._servo_type = node.create_client(
            ServoCommandType, f"{servo_node}/switch_command_type", callback_group=callback_group)

    def _call(self, client, request, timeout):
        if not client.wait_for_service(timeout_sec=timeout):
            raise RuntimeError(f"Service {client.srv_name} not available. Is the driver running?")
        result = wait_for_future(client.call_async(request), timeout)
        if result is None:
            raise RuntimeError(f"Service {client.srv_name} timed out")
        return result

    def states(self, timeout=5.0):
        """Return {controller_name: state} e.g. {'forward_position_controller': 'inactive'}."""
        result = self._call(self._list, ListControllers.Request(), timeout)
        return {c.name: c.state for c in result.controller}

    def active_mode(self):
        """'trajectory', 'servo', another active motion controller's name, or None."""
        states = self.states()
        for mode, name in MODES.items():
            if states.get(name) == "active":
                return mode
        for name in MOTION_CONTROLLERS:
            if states.get(name) == "active":
                return name
        return None

    def switch_to(self, mode, timeout=5.0):
        """Activate 'trajectory' or 'servo' and deactivate any other motion controller."""
        target = MODES[mode]
        states = self.states(timeout)
        if target not in states:
            raise RuntimeError(f"Controller {target} is not loaded")

        deactivate = [n for n in MOTION_CONTROLLERS if n != target and states.get(n) == "active"]
        if states[target] != "active" or deactivate:
            req = SwitchController.Request()
            req.activate_controllers = [] if states[target] == "active" else [target]
            req.deactivate_controllers = deactivate
            req.strictness = SwitchController.Request.STRICT
            req.activate_asap = True
            req.timeout.sec = int(timeout)
            if not self._call(self._switch, req, timeout).ok:
                raise RuntimeError(f"Switching to {target} failed (see controller_manager log)")
            self._node.get_logger().info(f"Active motion controller: {target}")

        if mode == "servo":
            self.set_servo_command_type(ServoCommandType.Request.TWIST, timeout)

    def set_servo_command_type(self, command_type=ServoCommandType.Request.TWIST, timeout=5.0):
        """Tell MoveIt Servo which input to listen to: JOINT_JOG=0, TWIST=1, POSE=2."""
        req = ServoCommandType.Request()
        req.command_type = command_type
        if not self._call(self._servo_type, req, timeout).success:
            raise RuntimeError("MoveIt Servo rejected the command type")
