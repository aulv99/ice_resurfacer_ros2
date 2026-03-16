# Autonomous ROS 2 Ice Resurfacer 

A work-in-progress ROS 2 (Jazzy) simulation of an autonomous ice resurfacer (Zamboni).

## Current Features
* **Custom URDF/Xacro:** Fully modeled chassis with Ackermann steering and custom ice-friction physics.
* **Gazebo Sim Integration:** Simulated in the modern Gazebo environment with a custom `.stl` hockey rink.
* **Sensor Suite:** Equipped with a 180-degree front-facing Lidar (to see the boards) and an IMU.
* **Sensor Fusion (EKF):** Uses `robot_localization` to fuse wheel odometry and IMU data, preventing the robot from getting lost when the wheels inevitably slip on the ice.

## Up Next
* Implementing Nav2 with the Smac Planner (Ackermann).
* Writing a custom Python commander to execute the traditional overlapping "Zamboni route" pattern


```bash
# Build the workspace
cd ~/ice_resurfacer_ws
colcon build
source install/setup.bash

# Launch the simulation, robot state publisher, controllers, and EKF
ros2 launch ice_resurfacer_gazebo simulation.launch.py