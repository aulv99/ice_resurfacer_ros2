# Autonomous ROS 2 Ice Resurfacer Simulation

An advanced, full-stack ROS 2 (Jazzy) simulation of an autonomous ice resurfacer (Zamboni). This project demonstrates high-precision localization, hybrid path planning, and custom control architectures required for autonomous operation in a constrained, geometrically strict environment.

## Key Features & Architecture

### Kinematics & Physics Simulation
* **Accurate Vehicle Dynamics:** Custom URDF/Xacro featuring true Ackermann steering with the base_link correctly anchored to the rear axle for mathematically accurate motion planning.
* **Gazebo Sim Integration:** Simulated in the modern Gazebo environment, complete with a custom 60m x 30m ice rink, garage, and snow-unloading pit.
* **Tuned Friction Models:** Realistic simulated wheel-on-ice slip characteristics handled via gz_ros2_control

### Robust Localization Stack
* **Sensor Fusion:** Utilizes an Extended Kalman Filter (EKF) to fuse high-frequency IMU data with wheel encoder odometry.
* **AMCL Integration:** Adaptive Monte Carlo Localization relies on a 180-degree front-facing GPU Lidar to continuously correct odometry drift against the rink's perimeter boards, ensuring millimeter-level accuracy in a feature-sparse environment.

### Hybrid Navigation System
* **Transit & Maneuvering (Nav2):** Leverages the ROS 2 Navigation Stack (SmacPlannerHybrid) utilizing Dubin motion model to dynamically plan complex routes out of the garage, through tight gates, and reverse out of the snow unloading pit.
* **Resurfacing (Custom Pure Pursuit):** Bypasses Nav2 for the actual resurfacing phase. A custom Python-based Pure Pursuit controller actively queries the TF2 tree (map -> base_link) to track algorithmically generated paths, adjusting dynamic lookahead based on true velocity.

### Algorithmic Path Generation
* **Geometric Rink Coverage:** Programmatically calculates classic Zamboni sweeping patterns based on customizable rink dimensions and lane spacing.
* **Dynamic Corner Tapers:** Calculates real-time lateral offsets to prevent collisions. As the vehicle enters 8.5m corners, the trajectory dynamically shifts outward to account for Ackermann "nose swing" and conditioner "tail swing."

### Real-Time Monitoring & Subsystems
* **Live Coverage Tracking:** A highly optimized background node applies a boolean rink mask over a custom OccupancyGrid, painting a live heatmap of cleaned ice in RViz and broadcasting a precise completion percentage (0.0% to 100.0%) to custom Web UI.
* **Conditioner Management:** A dedicated node manages physical resurfacing hardware. It maps water valve openings proportionally to the vehicle's true velocity, simulates water/fuel depletion, and commands the augers and conditioner lift via FollowJointTrajectory actions.

### A Custom Monitoring and Control UI 
* **Modern Web Dashboard:** A lightweight, front-end interface built with a streamlined HTML, CSS, and JavaScript architecture. Connected directly to the ROS 2 network via roslibjs, it features a sleek aesthetic with dedicated views for the main operational dashboard, active alarm monitoring, hardware trends, and real-time autonomous sequence tracking.


## Prerequisites 
* **OS:** Ubuntu 24.04 (Recommended)
* **ROS 2:** Jazzy Jalisco
* **Simulation:** Gazebo (Harmonic)
* **Dependencies:** nav2, ros2_control, gz_ros2_control, robot_localization

## Build and Run Instructions

```bash
# building workspace
cd ~/ros2_ws 
colcon build --symlink-install
source install/setup.bash

# Launch simulation (Gazebo)
ros2 launch ice_resurfacer_gazebo autonomy.launch.py

# Launch visualization (RViz)
ros2 launch ice_resurfacer_description visualize.launch.py

# Launch autonomy stack
ros2 launch ice_resurfacer_control autonomy_stack.launch.py

# Launch UI
# 1st terminal
ros2 launch rosbridge_server rosbridge_websocket_launch.xml
# 2nd terminal
cd src/ice_resurfacer_ros2/ice_resurfacer_ui
python3 -m http.server 8000

# start sequence 
# Press "Aloita jääajo" from UI or use ros2 service call from terminal
ros2 service call /start_sequence std_srvs/srv/Trigger {}