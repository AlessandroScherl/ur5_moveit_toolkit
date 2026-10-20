"""
High-level UR5 interface on top of MoveIt 2 (pymoveit2), MoveIt Servo and TF2.

    from ur5_moveit_toolkit import UR5

    robot = UR5()
    print(robot.joints())                 # 6 joint angles [rad]
    print(robot.tcp_pose())               # [x, y, z, rx, ry, rz] of tool0 in 'base'
    robot.move_joints([0, -1.57, 1.57, -1.57, -1.57, 0])
    robot.move_relative(dz=-0.05)         # 5 cm down in base frame (straight line)
    robot.move_relative(dz=0.03, frame="tool")  # 3 cm along the tool Z axis
    robot.jog(linear=(0, 0, 0.02), duration=1.0)  # MoveIt Servo, 2 cm/s for 1 s
    robot.shutdown()

Pose format is the same as on the UR teach pendant / ur_rtde: position in metres,
orientation as a rotation vector in radians, expressed in the `base` frame.
NOTE: these are tool0 (flange) poses. The pendant shows the TCP *including* the TCP
offset configured on the robot, so the numbers only match if the pendant TCP is zero.
"""
import time
from threading import Thread

import numpy as np
import rclpy
from geometry_msgs.msg import TwistStamped
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.time import Duration, Time
from pymoveit2 import MoveIt2, MoveIt2State
from scipy.spatial.transform import Rotation
from tf2_ros import Buffer, TransformListener

from .controllers import ControllerSwitcher, wait_for_future

JOINT_NAMES = [
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
]
MOVE_GROUP = "ur_manipulator"
# "base" matches the teach pendant's Base frame. "base_link" is rotated 180 deg about Z!
BASE_FRAME = "base"
TOOL_FRAME = "tool0"
SERVO_TWIST_TOPIC = "/servo_node/delta_twist_cmds"


class UR5:
    def __init__(self, node_name="ur5_toolkit", wait_for_joint_states=True):
        self._owns_rclpy = not rclpy.ok()
        if self._owns_rclpy:
            rclpy.init()

        self.node = Node(node_name)
        self._cb = ReentrantCallbackGroup()

        self.moveit2 = MoveIt2(
            node=self.node,
            joint_names=JOINT_NAMES,
            base_link_name=BASE_FRAME,
            end_effector_name=TOOL_FRAME,
            group_name=MOVE_GROUP,
            callback_group=self._cb,
        )
        self.controllers = ControllerSwitcher(self.node, self._cb)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self.node)
        self._twist_pub = self.node.create_publisher(TwistStamped, SERVO_TWIST_TOPIC, 10)

        # All ROS callbacks run in this background thread; the methods below only
        # wait/poll. (Do not call rclpy.spin*() on self.node yourself.)
        # Single-threaded on purpose: pymoveit2 + MultiThreadedExecutor busy-spins at
        # ~100% CPU and starves every other Python thread of the GIL (-> Servo commands
        # arrive late -> watchdog stops the robot -> stutter).
        self._executor = SingleThreadedExecutor()
        self._executor.add_node(self.node)
        self._spin_thread = Thread(target=self._executor.spin, daemon=True)
        self._spin_thread.start()

        if wait_for_joint_states:
            self.wait_for_joint_states()
        self.node.get_logger().info("UR5 interface ready")

    # ------------------------------------------------------------------ state
    def wait_for_joint_states(self, timeout=10.0):
        deadline = time.monotonic() + timeout
        while self.joints() is None:
            if time.monotonic() > deadline:
                raise RuntimeError("No /joint_states received. Is ur5_driver.launch.py running?")
            time.sleep(0.05)

    def joints(self):
        """Current joint angles [rad] in UR order (base ... wrist 3), or None."""
        js = self.moveit2.joint_state
        if js is None:
            return None
        # /joint_states is NOT published in UR order -> look up by name
        try:
            return [js.position[list(js.name).index(n)] for n in JOINT_NAMES]
        except ValueError:
            return None

    def tcp_pose(self, frame=BASE_FRAME, link=TOOL_FRAME, timeout=2.0):
        """[x, y, z, rx, ry, rz] (metres, rotation vector) of `link` in `frame`."""
        tf = self.tf_buffer.lookup_transform(frame, link, Time(), timeout=Duration(seconds=timeout))
        t, q = tf.transform.translation, tf.transform.rotation
        rotvec = Rotation.from_quat([q.x, q.y, q.z, q.w]).as_rotvec()
        return [t.x, t.y, t.z, *rotvec]

    # ----------------------------------------------------------- controllers
    def use_trajectory_mode(self):
        """Activate scaled_joint_trajectory_controller (needed for move_* methods)."""
        self.controllers.switch_to("trajectory")

    def use_servo_mode(self):
        """Activate forward_position_controller + Servo twist input (needed for jog)."""
        self.controllers.switch_to("servo")

    # --------------------------------------------------------- planned moves
    def move_joints(self, joints, speed=0.2, accel=0.2, timeout=60.0):
        """Plan + execute a joint-space move. speed/accel are 0..1 scaling factors."""
        return self._plan_and_execute(speed, accel, timeout,
                                      joint_positions=list(joints), joint_names=JOINT_NAMES)

    def move_pose(self, pose, cartesian=False, speed=0.1, accel=0.1, timeout=60.0):
        """Move tool0 to [x, y, z, rx, ry, rz] in `base`.
        cartesian=True: straight-line path (fails if not fully reachable).
        cartesian=False: any collision-free path; the arm may swing around."""
        quat = Rotation.from_rotvec(pose[3:6]).as_quat().tolist()
        return self._plan_and_execute(speed, accel, timeout,
                                      position=list(pose[:3]), quat_xyzw=quat,
                                      frame_id=BASE_FRAME, cartesian=cartesian)

    def move_relative(self, dx=0.0, dy=0.0, dz=0.0, drx=0.0, dry=0.0, drz=0.0,
                      frame="base", cartesian=True, speed=0.1, accel=0.1):
        """Offset the current tool0 pose. frame='base' or 'tool' (tool0 axes).
        Rotations (rad) are applied about the axes of the same frame."""
        pose = self.tcp_pose()
        pos, rot = np.array(pose[:3]), Rotation.from_rotvec(pose[3:])
        d_pos, d_rot = np.array([dx, dy, dz]), Rotation.from_rotvec([drx, dry, drz])
        if frame == "tool":
            pos, rot = pos + rot.apply(d_pos), rot * d_rot
        elif frame == "base":
            pos, rot = pos + d_pos, d_rot * rot
        else:
            raise ValueError("frame must be 'base' or 'tool'")
        return self.move_pose([*pos, *rot.as_rotvec()], cartesian=cartesian,
                              speed=speed, accel=accel)

    def _plan_and_execute(self, speed, accel, timeout, cartesian=False, **goal):
        self.use_trajectory_mode()
        self.moveit2.max_velocity = float(np.clip(speed, 0.01, 1.0))
        self.moveit2.max_acceleration = float(np.clip(accel, 0.01, 1.0))
        log = self.node.get_logger()

        # Plan (pymoveit2's blocking plan() would spin our node a second time)
        future = self.moveit2.plan_async(
            cartesian=cartesian, start_joint_state=self.moveit2.joint_state, **goal)
        if future is None or wait_for_future(future, 15.0) is None:
            log.error("Planning request failed or timed out (is move_group running?)")
            return False
        trajectory = self.moveit2.get_trajectory(
            future, cartesian=cartesian, cartesian_fraction_threshold=0.95 if cartesian else 0.0)
        if trajectory is None:
            log.error("Planning failed: target unreachable or in collision")
            return False

        # Execute (pymoveit2 would deadlock on a not-yet-ready action server)
        if not self.moveit2._execute_trajectory_action_client.wait_for_server(timeout_sec=5.0):
            log.error("execute_trajectory action server not available")
            return False
        self.moveit2.execute(trajectory)
        deadline = time.monotonic() + timeout
        while self.moveit2.query_state() != MoveIt2State.IDLE:
            if time.monotonic() > deadline:
                log.error("Execution timed out, cancelling")
                self.moveit2.cancel_execution()
                return False
            time.sleep(0.01)
        if not self.moveit2.motion_suceeded:
            log.error(f"Execution failed: {self.moveit2.get_last_execution_error_code()}")
        return self.moveit2.motion_suceeded

    # ----------------------------------------------------------------- servo
    def send_twist(self, linear=(0.0, 0.0, 0.0), angular=(0.0, 0.0, 0.0), frame=TOOL_FRAME):
        """Send ONE velocity command (m/s, rad/s) to MoveIt Servo. Call this at >= 50 Hz;
        Servo stops the robot if commands stop arriving (incoming_command_timeout)."""
        msg = TwistStamped()
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.header.frame_id = frame
        msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z = map(float, linear)
        msg.twist.angular.x, msg.twist.angular.y, msg.twist.angular.z = map(float, angular)
        self._twist_pub.publish(msg)

    def jog(self, linear=(0.0, 0.0, 0.0), angular=(0.0, 0.0, 0.0), duration=1.0,
            frame=TOOL_FRAME, rate=100.0):
        """Move with a constant velocity for `duration` seconds via MoveIt Servo."""
        self.use_servo_mode()
        end = time.monotonic() + duration
        try:
            while time.monotonic() < end:
                self.send_twist(linear, angular, frame)
                time.sleep(1.0 / rate)
        finally:
            for _ in range(10):
                self.send_twist(frame=frame)
                time.sleep(1.0 / rate)

    # --------------------------------------------------------- planning scene
    def add_box(self, name, size, position, quat_xyzw=(0.0, 0.0, 0.0, 1.0), frame=BASE_FRAME):
        """Add a collision box (e.g. the table) so MoveIt plans around it."""
        self.moveit2.add_collision_box(id=name, size=tuple(size), position=tuple(position),
                                       quat_xyzw=tuple(quat_xyzw), frame_id=frame)

    def remove_object(self, name):
        self.moveit2.remove_collision_object(name)

    # --------------------------------------------------------------- cleanup
    def shutdown(self):
        self._executor.shutdown(timeout_sec=2.0)  # lets pending callbacks finish first
        self._spin_thread.join()
        self.node.destroy_node()
        if self._owns_rclpy:
            rclpy.try_shutdown()

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.shutdown()
