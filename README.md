# Autonomous ROS 2 Ice Resurfacer 

A work-in-progress ROS 2 (Jazzy) simulation of an autonomous ice resurfacer (Zamboni).

## Current Features
* **Custom URDF/Xacro:** Fully modeled chassis with Ackermann steering and custom ice-friction physics.
* **Gazebo Sim Integration:** Simulated in the modern Gazebo environment with a custom ice hall world model
* **Sensor Suite:** Equipped with a 180-degree front-facing Lidar (to see the boards) and an IMU.
* **Sensor Fusion (AMCL / EKF):** Prepared to utilize Adaptive Monte Carlo Localization to localize based on SLAM obtained MAP
* **Path Planning:** Dynamic entering sequence, static and geometric surfacing pattern based on rink dimensions, and dynamic multi-part exit and unloading sequence




```bash
# Build the workspace
cd ~/ice_resurfacer_ws
colcon build
source install/setup.bash

# Launch the simulation
ros2 launch ice_resurfacer_gazebo autonomy.launch.py