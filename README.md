# Dummy Spider Robot Simulation

A 12-DOF quadruped robot simulation built with **ROS 2 Jazzy Jalisco** and **Gazebo Harmonic**. This project demonstrates multi-legged locomotion, featuring a stable 360-degree rotation (turning in place) and a coordinated forward walking gait using diagonal-pair synchronization.

---

## Locomotion Demonstrations

### 1. 360° Spin (Rotation on Axis)
The robot coordinates diagonal pairs to rotate the base plate while maintaining a stable center of mass.

Camera View 
[spin_camera.webm](https://github.com/user-attachments/assets/6a672424-7f4e-47fe-a1cd-0d11a21393b2)

3rd Person View
[spin_3p.webm](https://github.com/user-attachments/assets/77e3b3b8-fbbb-4029-b53e-ff499c885066)

### 2. Forward Walk
The robot uses a stepping gait to move toward the front. The legs push backward while in contact with the ground (stance) and swing forward while lifted (swing).

Camera View 
[walk_camera.webm](https://github.com/user-attachments/assets/16852040-6140-405d-a2e2-6b0de0ae403d)
3rd Person View
[walk_camera.webm](https://github.com/user-attachments/assets/e422677f-d7ff-4697-b2e3-a778e9e2fc8b)

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
