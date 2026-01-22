# Dummy Spider Robot Simulation

A 12-DOF quadruped robot simulation built with **ROS 2 Jazzy Jalisco** and **Gazebo Harmonic**. This project demonstrates multi-legged locomotion, featuring a stable 360-degree rotation (turning in place) and a coordinated forward walking gait using diagonal-pair synchronization.

---

## Locomotion Demonstrations

### 1. 360° Spin (Rotation on Axis)
The robot coordinates diagonal pairs to rotate the base plate while maintaining a stable center of mass.

| Camera View | 3rd Person View |
| :---: | :---: |
| ![spin_camera](https://github.com/user-attachments/assets/7ba4ba52-1c63-4e49-9340-8d0d55a76c89) | ![spin_3p](https://github.com/user-attachments/assets/45ac9200-c7da-4a90-b4b1-2263e5c8be55) |

### 2. Forward Walk
The robot uses a stepping gait to move toward the front. The legs push backward while in contact with the ground (stance) and swing forward while lifted (swing).

| Camera View | 3rd Person View |
| :---: | :---: |
| ![Walk Cam](videos/walk_camera.gif) | ![Walk 3rd Person](videos/walk_3p.gif) |

---

## Run Commands

### 1. Setup & Build
```bash
# Install dependencies
sudo apt install ros-jazzy-ros-gz ros-jazzy-gz-ros2-control ros-jazzy-ros2-control ros-jazzy-ros2-controllers ros-jazzy-xacro

# Build the workspace
cd ~/ros2_ws
colcon build --packages-select dummy_spider
source install/setup.bash
```

### 2. Launch Simulation
This starts the Gazebo world, spawns the robot, and loads the controllers after an 8-second delay.
```bash
ros2 launch dummy_spider dummy_spider.launch.py
```

### 3. Execute Nodes
In a new terminal (sourced with your workspace), run one of the following commands to move the spider:

#### To Rotate 360° on Axis:
```bash
python3 src/dummy_spider/dummy_spider/spider_node.py
```
#### To Walk Forward:
```bash
python3 src/dummy_spider/dummy_spider/spider_walk.py
```
