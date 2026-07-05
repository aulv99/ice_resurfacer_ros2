#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float64MultiArray
from geometry_msgs.msg import Twist
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint

class ConditionerManager(Node):
    def __init__(self):
        super().__init__('conditioner_manager')
        
        self.get_logger().info("Conditioner Manager Node starting up...")

        # ---------------------------------------------------------
        # STATE TRACKING
        # ---------------------------------------------------------
        self.current_mission_state = 'IDLE'
        self.MAX_WATER_VALVE_OPENING = 1.57 # 90 degrees in radians
        self.MAX_AUGER_SPEED = 15.0         # rad/s

        # ---------------------------------------------------------
        # SUBSCRIBERS (Listening to the Master Node & Chassis)
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

        # ---------------------------------------------------------
        # ACTION CLIENT (Migrated from autonomy_seq.py)
        # ---------------------------------------------------------
        self._conditioner_client = ActionClient(
            self, 
            FollowJointTrajectory, 
            '/conditioner_controller/follow_joint_trajectory'
        )

    # ============================================================
    # CALLBACKS & LOGIC
    # ============================================================

    def state_callback(self, msg):
        """ Triggered whenever autonomy_seq.py broadcasts a new phase """
        previous_state = self.current_mission_state
        self.current_mission_state = msg.data

        if self.current_mission_state == 'RESURFACING' and previous_state != 'RESURFACING':
            self.get_logger().info("Conditioner engaged: Dropping blade and starting augers.")
            self.set_conditioner_lift(0.2)  # Drop blade
            # self.set_auger_speed(self.MAX_AUGER_SPEED)
            
        elif self.current_mission_state in ['TRANSITING_EXIT', 'IDLE'] and previous_state == 'RESURFACING':
            self.get_logger().info("Conditioner disengaged: Lifting blade, stopping augers, shutting water.")
            self.set_conditioner_lift(-0.2) # Lift blade
            # self.set_auger_speed(0.0)
            # self.set_water_valve(0.0)

    def velocity_callback(self, msg):
        """ Feedforward control for the water valve based on vehicle speed and auger control"""
        if self.current_mission_state == 'RESURFACING':
            current_speed = msg.linear.x
            MAX_VEHICLE_SPEED = 2.0
            speed_ratio = min(current_speed / MAX_VEHICLE_SPEED, 1.0)
            # Map to the 90-degree valve opening
            valve_position = speed_ratio * self.MAX_WATER_VALVE_OPENING
            
            self.set_water_valve(valve_position)
            self.set_auger_speed(self.MAX_AUGER_SPEED)

        else:
            self.set_water_valve(0.0)
            self.set_auger_speed(0.0)
    # ============================================================
    # HARDWARE COMMAND HELPERS
    # ============================================================

    def set_auger_speed(self, speed):
        """ Publishes the target velocity to both horizontal and vertical augers """
        msg = Float64MultiArray()
        msg.data = [float(speed), float(speed)]
        self.auger_pub.publish(msg)

    def set_water_valve(self, position_rad):
        """ Publishes the target position (0.0 to 1.57) to the water valve """
        msg = Float64MultiArray()
        msg.data = [float(position_rad)]
        self.water_pub.publish(msg)

    def set_conditioner_lift(self, target_position, duration_sec=3):
        """ Handles the heavy mechanical lift. Exactly as it was in your old script. """
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

def main(args=None):
    rclpy.init(args=args)
    node = ConditionerManager()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()