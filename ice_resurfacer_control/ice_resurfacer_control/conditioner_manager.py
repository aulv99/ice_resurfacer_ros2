#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float64MultiArray, Float64
from geometry_msgs.msg import Twist
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint

class ConditionerManager(Node):
    def __init__(self):
        super().__init__('conditioner_manager')
        
        self.get_logger().info("Jäänhoitolaitteisto käynnistyy...")

        # ---------------------------------------------------------
        # STATE TRACKING
        # ---------------------------------------------------------
        self.current_mission_state = 'IDLE'
        self.MAX_WATER_VALVE_OPENING = 1.00 # 0-100 % Open
        self.MAX_AUGER_SPEED = 15.0         # rad/s
        self.MAX_TANK_CAPACITY = 727.0
        self.MAX_FUEL_CAPACITY = 100.0
        self.current_fuel_capacity = self.MAX_FUEL_CAPACITY      
        self.current_water_capacity = self.MAX_TANK_CAPACITY
        self.WATER_FLOW_RATE = 0.5
        self.current_valve_opening = 0.0
        self.current_velocity = 0.0

        # --------------------------------
        # Control Loop
        # --------------------------------
        self.dt = 0.1  # 0.1 seconds = 10Hz loop rate
        self.timer = self.create_timer(self.dt, self.control_loop)

        # ---------------------------------------------------------
        # SUBSCRIBERS 
        # ---------------------------------------------------------
        # Listens to autonomy_seq.py for state changes (e.g., 'RESURFACING')
        self.state_sub = self.create_subscription(
            String,
            '/mission_state',
            self.state_callback,
            10
        )
        
        # Listens to cmd_vel to calculate water flow proportionally
        self.vel_sub = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.velocity_callback,
            10
        )

        # ---------------------------------------------------------
        # PUBLISHERS (Commanding the ros2_control hardware)
        # ---------------------------------------------------------
        self.auger_pub = self.create_publisher(
            Float64MultiArray, 
            '/auger_velocity_controller/commands', 
            10
        )
        
        self.water_pub = self.create_publisher(
            Float64MultiArray, 
            '/water_valve_controller/commands', 
            10
        )

        self.tank_level_pub = self.create_publisher(
            Float64,
            '/water_tank_level',
            10
        )

        self.fuel_level_pub = self.create_publisher(
            Float64,
            '/fuel_tank_level',
            10
        )

        # ---------------------------------------------------------
        # ACTION CLIENT 
        # ---------------------------------------------------------
        self._conditioner_client = ActionClient(
            self, 
            FollowJointTrajectory, 
            '/conditioner_controller/follow_joint_trajectory'
        )

    # ============================================================
    # CALLBACKS & LOGIC
    # ============================================================

    # Triggered whenever autonomy_seq.py broadcasts a new phase
    def state_callback(self, msg):
        previous_state = self.current_mission_state
        self.current_mission_state = msg.data

        if self.current_mission_state == 'PHASE_2' and previous_state != 'PHASE_2':
            self.get_logger().info("Lasketaan jäädytin ja käynnistetään lumikairat.")
            self.set_conditioner_lift(0.2)  # Drop blade
            # self.set_auger_speed(self.MAX_AUGER_SPEED)
            
        elif self.current_mission_state in ['PHASE_3A', 'IDLE'] and previous_state == 'PHASE_2':
            self.get_logger().info("Nostetaan jäädytin, pysäytetään lumikairat, ja katkaistaan vesisyöttö")
            self.set_conditioner_lift(-0.2) # Lift blade
            # self.set_auger_speed(0.0)
            # self.set_water_valve(0.0)

    def velocity_callback(self, msg):
        """ Feedforward control for the water valve based on vehicle speed and auger control"""
        if self.current_mission_state == 'PHASE_2':
            current_speed = abs(msg.linear.x) # Added absolute value to avoid issues reversing
            MAX_VEHICLE_SPEED = 2.0
            speed_ratio = min(current_speed / MAX_VEHICLE_SPEED, 1.0)
            # Map to the 90-degree valve opening
            valve_position = speed_ratio * self.MAX_WATER_VALVE_OPENING
            self.current_valve_opening = speed_ratio
            self.current_velocity = current_speed
            
            self.set_water_valve(valve_position)
            self.set_auger_speed(self.MAX_AUGER_SPEED)

        else:
            self.set_water_valve(0.0)
            self.set_auger_speed(0.0)
    # ============================================================
    # HARDWARE COMMAND HELPERS
    # ============================================================

    # Publishes the target velocity to both horizontal and vertical augers
    def set_auger_speed(self, speed):
        msg = Float64MultiArray()
        msg.data = [float(speed), float(speed)]
        self.auger_pub.publish(msg)

    # Publishes the target position (0.0 to 1.00) to the water valve 
    def set_water_valve(self, position_rad):
        msg = Float64MultiArray()
        msg.data = [float(position_rad)]
        self.water_pub.publish(msg)

    # Executes the conditioner movement
    def set_conditioner_lift(self, target_position, duration_sec=3):
        if not self._conditioner_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().error('Conditioner Action Server not available!')
            return

        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory.joint_names = ['conditioner_joint']
        point = JointTrajectoryPoint()
        point.positions = [float(target_position)]
        point.time_from_start.sec = int(duration_sec)
        
        goal_msg.trajectory.points = [point]
        self._conditioner_client.send_goal_async(goal_msg)

    # Control loop that runs automatically every 0.1 seconds
    def control_loop(self):
        
        # Calculate how much water we sprayed in the last 0.1 seconds
        water_consumed = self.current_valve_opening * self.WATER_FLOW_RATE * self.dt
        fuel_consumed = self.current_velocity * 0.01 * self.dt
        
        # Drain the tank (using max() so it never goes below 0)
        self.current_water_capacity = max(self.current_water_capacity - water_consumed, 0.0)
        self.current_fuel_capacity = max(self.current_fuel_capacity - fuel_consumed, 0.0)
        
        # Calculate the percentage for the UI (0.0 to 100.0)
        tank_percentage = (self.current_water_capacity / self.MAX_TANK_CAPACITY) * 100.0
        fuel_percentage = (self.current_fuel_capacity / self.MAX_FUEL_CAPACITY) * 100.0
        
        # Level alarms
        if tank_percentage <= 50.0 and not self.water_50_warned:
            self.get_logger().info("Vesisäiliön taso 50%")
            self.water_50_warned = True
        
        if fuel_percentage == 15.0 and not self.fuel_15_warned:
            self.get_logger().info("Polttoaine lopussa. Tankkaa ajoneuvo.")
            self.fuel_15_warned = True

        # 4. Publish it to the ROS 2 network
        msg = Float64()
        msg.data = tank_percentage
        self.tank_level_pub.publish(msg)

        fuel_msg = Float64()
        fuel_msg.data = fuel_percentage
        self.fuel_level_pub.publish(fuel_msg)

def main(args=None):
    rclpy.init(args=args)
    node = ConditionerManager()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()