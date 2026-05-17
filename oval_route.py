#!/usr/bin/env python3
import rclpy
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from geometry_msgs.msg import PoseStamped
import math

def create_pose(navigator, x, y, yaw):
    pose = PoseStamped()
    pose.header.frame_id = 'map'
    pose.header.stamp = navigator.get_clock().now().to_msg()
    pose.pose.position.x = float(x)
    pose.pose.position.y = float(y)
    pose.pose.position.z = 0.0
    
    pose.pose.orientation.z = math.sin(yaw / 2.0)
    pose.pose.orientation.w = math.cos(yaw / 2.0)
    return pose

def main():
    rclpy.init()
    navigator = BasicNavigator()

    print("Waiting for Nav2...")
    navigator.waitUntilNav2Active(localizer='bt_navigator')
    print("Generating Safe Oval Track with Merge Sequence...")

    route = []
    
    # Track Dimensions
    straight_length = 15.0
    turn_radius = 8.0

    # --- 1. THE MERGE (Lead-in Sequence) ---
    # Drive straight ahead slightly to get moving
    route.append(create_pose(navigator, 4.0, 0.0, 0.0))
    # Smoothly shift right down into the bottom lane
    route.append(create_pose(navigator, 10.0, -turn_radius, 0.0))

    # --- 2. BOTTOM STRAIGHTAWAY ---
    route.append(create_pose(navigator, straight_length, -turn_radius, 0.0))

    # --- 3. RIGHT TURN ---
    center_x = straight_length
    center_y = 0.0
    # Added -90 and 90 to make the corner perfectly round
    for angle_deg in [-90, -45, 0, 45, 90]:
        angle_rad = math.radians(angle_deg)
        x = center_x + (turn_radius * math.cos(angle_rad))
        y = center_y + (turn_radius * math.sin(angle_rad))
        yaw = angle_rad + (math.pi / 2) # Tangent to the curve
        route.append(create_pose(navigator, x, y, yaw))

    # --- 4. TOP STRAIGHTAWAY ---
    route.append(create_pose(navigator, -straight_length, turn_radius, math.pi))

    # --- 5. LEFT TURN ---
    center_x = -straight_length
    center_y = 0.0
    for angle_deg in [90, 135, 180, 225, 270]:
        angle_rad = math.radians(angle_deg)
        x = center_x + (turn_radius * math.cos(angle_rad))
        y = center_y + (turn_radius * math.sin(angle_rad))
        yaw = angle_rad + (math.pi / 2) 
        route.append(create_pose(navigator, x, y, yaw))

    # --- 6. FINISH LAP ---
    # Bring it back to the center of the bottom straightaway
    route.append(create_pose(navigator, 0.0, -turn_radius, 0.0))

    print(f"Sending {len(route)} waypoints to Nav2...")
    navigator.goThroughPoses(route)

    while not navigator.isTaskComplete():
        pass # Let it drive!

    result = navigator.getResult()
    if result == TaskResult.SUCCEEDED:
        print("Lap Complete!")
    else:
        print("Lap Failed or Canceled.")

    navigator.lifecycleShutdown()
    rclpy.shutdown()

if __name__ == '__main__':
    main()