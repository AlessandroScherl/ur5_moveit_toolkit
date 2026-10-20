# 2. Robot & network setup

## 2.1 Network

The laptop talks to the UR controller over Ethernet (TCP ports 29999, 30001–30004, and
50001–50004 back to the laptop).

| | Lab value (ART-VS setup) |
|---|---|
| Robot IP | `10.0.0.30` (pendant: *Setup Robot → Network*) |
| Laptop | wired NIC in the same subnet, e.g. `10.0.0.x/24` (static or lab DHCP) |

Static IP for the laptop (replace `eno1` with your interface from `ip -brief link`):

```bash
sudo nmcli connection add type ethernet ifname eno1 con-name ur5 \
     ipv4.method manual ipv4.addresses 10.0.0.37/24
sudo nmcli connection up ur5
ping -c 3 10.0.0.30
```

Choose an address nobody else uses. Wi-Fi can stay on for internet access. Disable
the firewall for that interface, or allow ports 50001–50004 (the robot connects *back* to
the laptop).

## 2.2 On the teach pendant

1. Power on → *Initialize* → **ON** → **START** (brakes released).
2. Check *Installation → General → TCP*: the toolkit works with `tool0` (the flange).
   The pendant's displayed pose includes the TCP offset you configure there.
3. Speed slider (bottom of the *Run*/*Move* screen): start at **10–30 %**. The
   `scaled_joint_trajectory_controller` respects it.

### Option A — headless mode (default in this toolkit)

`ur5_driver.launch.py headless_mode:=true` makes the driver upload its control script
directly. Nothing to do on the pendant except having the robot running. CB3 robots
like our UR5 accept this as long as no other program is playing.

After a protective stop or E-stop the script stops. Unlock on the pendant, then:

```bash
ros2 service call /io_and_status_controller/resend_robot_program std_srvs/srv/Trigger
```

### Option B — External Control URCap (pendant program)

Use this if headless mode is not allowed in your lab, or to combine ROS with pendant
programs.

1. Install the **External Control** URCap (`externalcontrol-x.y.z.urcap` from
   <https://github.com/UniversalRobots/Universal_Robots_ExternalControl_URCap/releases>):
   copy it to a USB stick → *Setup Robot → URCaps → +* → restart.
2. *Installation → URCaps → External Control*: **Host IP** = laptop IP, **port** 50002.
3. Make a program containing the *External Control* node and save it (e.g. `ros_control.urp`).
4. Start the driver with `headless_mode:=false`, then press **Play** on the pendant.
   The driver log shows `Robot connected to reverse interface. Ready to receive control commands.`

On e-Series robots (UR5e etc.) you also need *Settings → System → Remote Control → Enable*,
and the pendant set to *Remote* for headless mode. For these robots pass `ur_type:=ur5e`
and `camera_mount:=false` (or your own URDF).

## 2.3 Dashboard commands from the laptop (optional)

The driver starts a dashboard client, so you can power the robot without touching the pendant:

```bash
ros2 service call /dashboard_client/power_on   std_srvs/srv/Trigger
ros2 service call /dashboard_client/brake_release std_srvs/srv/Trigger
ros2 service call /dashboard_client/unlock_protective_stop std_srvs/srv/Trigger
ros2 service call /dashboard_client/play       std_srvs/srv/Trigger   # Option B program
ros2 service call /dashboard_client/stop       std_srvs/srv/Trigger
```

## 2.4 Gripper (Robotiq 2F via USB/RS-485)

* Adapter appears as `/dev/ttyUSB0` (`ls /dev/ttyUSB*`).
* Your user must be in the `dialout` group (`install_dependencies.sh` does this; log out
  and in once).
* Test: `ros2 run ur5_moveit_toolkit gripper close` / `open`.

## 2.5 RealSense camera

* Plug into a USB **3** port (blue). `rs-enumerate-devices` should list it.
* `ros2 launch ur5_moveit_toolkit ur5_camera.launch.py fps:=15`
* Topics are double-namespaced in ROS 2: `/camera/camera/color/image_raw`,
  `/camera/camera/aligned_depth_to_color/image_raw`.
* View: `ros2 run rqt_image_view rqt_image_view`.
* TF: `ros2 run tf2_ros tf2_echo tool0 camera_color_optical_frame` should print a
  transform once driver and camera both run.
