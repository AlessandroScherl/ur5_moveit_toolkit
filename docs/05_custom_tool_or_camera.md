# 5. Custom tool, camera or URDF

## 5.1 What the default URDF contains

`urdf/ur5_with_realsense.urdf.xacro` = the standard `ur_robot_driver` description (UR
arm + ros2_control) **plus** the ART-VS camera bracket:

```
world → base_link → ... → tool0
                           └─ camera_fixture            (3D-printed bracket, mesh only)
                               └─ camera_bottom_screw_frame
                                   └─ camera_link        (D435 mesh)
                                       └─ camera_color_optical_frame  ← published by the RealSense driver
```

The bracket and camera have **visual geometry only**, no collision geometry. That avoids false
self-collision stops in MoveIt Servo. The downside is that MoveIt does not know the camera
is there. Keep it away from the table yourself, or add a simple collision box (5.3).

## 5.2 Making your own tool URDF

1. Copy the file: `cp urdf/ur5_with_realsense.urdf.xacro urdf/ur5_with_mytool.urdf.xacro`
2. Replace everything below the `RealSense D435i Camera Mount` comment with your links
   and fixed joints, parented to `tool0`:
   ```xml
   <link name="my_gripper">
     <visual><geometry><mesh filename="package://ur5_moveit_toolkit/urdf/meshes/my_gripper.stl" scale="0.001 0.001 0.001"/></geometry></visual>
     <collision><geometry><box size="0.08 0.08 0.15"/></geometry>
       <origin xyz="0 0 0.075"/></collision>
   </link>
   <joint name="tool0_my_gripper" type="fixed">
     <parent link="tool0"/><child link="my_gripper"/>
     <origin xyz="0 0 0" rpy="0 0 0"/>
   </joint>
   <link name="my_tcp"/>   <!-- fingertip centre -->
   <joint name="my_gripper_tcp" type="fixed">
     <parent link="my_gripper"/><child link="my_tcp"/><origin xyz="0 0 0.16"/>
   </joint>
   ```
   STL meshes from CAD are usually in mm → `scale="0.001 0.001 0.001"`.
3. Put meshes in `urdf/meshes/`, rebuild (`colcon build`), then launch with
   `description_file:=$(ros2 pkg prefix --share ur5_moveit_toolkit)/urdf/ur5_with_mytool.urdf.xacro`.
4. Check in RViz (*RobotModel* display) and with
   `ros2 run tf2_ros tf2_echo tool0 my_tcp`.

Measure the offsets carefully: for a camera the `tool0 → camera` transform directly
affects how accurate visual servoing and grasping are. A hand-eye calibration (e.g.
`easy_handeye2`) gives better values than CAD.

## 5.3 Collision objects at runtime

Instead of editing the URDF you can add boxes to MoveIt's planning scene:

```python
robot.add_box("table", size=(1.5, 1.5, 0.02), position=(0.0, 0.0, -0.02))
robot.add_box("wall",  size=(0.02, 2.0, 1.5),  position=(-0.6, 0.0, 0.75))
```

Objects are in the `base` frame and stay until `move_group` restarts or you call
`robot.remove_object("table")`.

## 5.4 Other UR models

The launch files take `ur_type:=ur3|ur5|ur10|ur5e|ur10e|...`. The camera URDF uses
the generic macros, so `ur_type` changes the arm; the bracket offsets are for our
bracket only. For a bare arm use `camera_mount:=false`.
