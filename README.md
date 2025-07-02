# Line Tracking Race Jazzy Version (Under Develop)
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
source /opt/ros/$ROS_DISTRO/setup.bash
sudo rosdep init
rosdep update
rosdep install --from-paths src --ignore-src -r -i -y --rosdistro <ROS_DISTRO>
colcon build --cmake-args -DBUILD_TESTING=ON
```

## 2. Usage
Type in the terminal:
```bash
ros2 launch ros_gz_example_bringup diff_drive.launch.py
```

## 3. Topics and Services
Explore using the ROS2 command line which information are available and design your high-level node. You can use the package `roos_gz_example_application` as container of your nodes.