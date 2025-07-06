# ROS Autonomous Car Package

![System Architecture](docs/system_architecture.png)

## Features
- Lane following with RealSense D456
- Obstacle avoidance with VLP-32 LiDAR
- Integrated navigation using LeGO-LOAM
- Arduino motor control interface

## Installation
```bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
git clone https://github.com/your-username/ros_autonomous_car.git
cd ~/catkin_ws
rosdep install --from-paths src --ignore-src -r -y
catkin_make
source devel/setup.bash
