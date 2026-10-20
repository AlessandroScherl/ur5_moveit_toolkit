# 1. Installation

Tested on: Ubuntu 24.04, ROS 2 Jazzy (apt), `ur_robot_driver` 3.7, MoveIt 2.12,
`pymoveit2` 4.2, UR5 (CB3), RealSense D435i.

## 1.1 ROS 2 Jazzy

Follow <https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html>
(`ros-jazzy-desktop`). Put `source /opt/ros/jazzy/setup.bash` in your `~/.bashrc`.

## 1.2 This package

```bash
mkdir -p ~/ur5_ws/src && cd ~/ur5_ws/src
git clone https://github.com/AlessandroScherl/ur5_moveit_toolkit.git
./ur5_moveit_toolkit/install_dependencies.sh
cd ~/ur5_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash          # add this line to ~/.bashrc too
```

Check:

```bash
ros2 pkg executables ur5_moveit_toolkit      # lists status, move_joints, jog, ...
```

`--symlink-install` means edits to launch files, configs and Python code take effect
without rebuilding. Rebuild after adding **new** files.

## 1.3 Conda / Python environments (read this if you use PyTorch etc.)

ROS 2 Jazzy's Python packages (`rclpy`, `pymoveit2`, `cv_bridge`) are built for the
**system Python 3.12**.

* **Simplest:** do *not* activate conda in the robot terminals. Build with system Python.
  Never run `colcon build` with a conda env active.
* **If your project needs a conda env** (deep-learning models, like ART-VS), create it
  with the *same* Python version and source ROS inside it:
  ```bash
  conda create -n myproj python=3.12
  conda activate myproj
  source /opt/ros/jazzy/setup.bash && source ~/ur5_ws/install/setup.bash
  pip install "numpy<2" scipy pyserial      # numpy 2 breaks cv_bridge
  ```
  Some pip packages (e.g. LangSAM) silently upgrade numpy to 2.x — downgrade again
  afterwards with `pip install "numpy<2"`.

## 1.4 Optional: real-time scheduling

The driver prints `Could not enable FIFO RT scheduling policy`. That is fine for MoveIt
and Servo at the speeds used here. For better timing:

```bash
sudo groupadd -f realtime && sudo usermod -aG realtime $USER
echo -e "@realtime soft rtprio 99\n@realtime hard rtprio 99" | sudo tee /etc/security/limits.d/realtime.conf
# log out and in
```

## 1.5 Optional: robot calibration (more accurate poses)

Every UR is factory-calibrated; the driver uses nominal kinematics unless you extract it.
With the robot connected:

```bash
ros2 launch ur_calibration calibration_correction.launch.py \
  robot_ip:=10.0.0.30 target_filename:="${HOME}/ur5_calibration.yaml"
# then always start the driver with:
ros2 launch ur5_moveit_toolkit ur5_driver.launch.py calibration_file:=${HOME}/ur5_calibration.yaml
```

Without it, TCP positions can be off by a few millimetres compared to the pendant.
