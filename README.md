# Line Tracking Race ROS2 Jazzy + Gazebo Harmonic Version  (Work in progress)
This repository collects the starting material for the Line Tracking Race project with Gazebo Harmonics and ROS2.

## 1. Install
### 1.1 Preliminaries
```bash
sudo apt install python3-vcstool python3-colcon-common-extensions git wget
```

### 1.2 Clone and Build
Clone in your workspace (e.g., `~/ros2_ws/src`):
```bash
git clone git@github.com:BRAIR-Education/line_tracking_race.git && git checkout jazzy
```
Build the simulation:
```bash
cd ~/ros2_ws
source /opt/ros/${ROS_DISTRO}/setup.bash
sudo rosdep init
rosdep update
rosdep install --from-paths src --ignore-src -r -i -y --rosdistro ${ROS_DISTRO}
colcon build
```

## 2. Usage
Type in the terminal:
```bash
ros2 launch line_tracking_race_bringup line_tracking_race.launch.py
```

## 3. Topics and Services
Explore using the ROS2 command line which information are available and design your high-level node. You can use the package `line_tracking_race_application` as container of your nodes.