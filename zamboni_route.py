import rclpy
from rclpy.node import Node
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from geometry_msgs.msg import PoseStamped
import math

# --- DEFINE YOUR SPAWN POINT HERE ---
SPAWN_X = 0 # -20.0
SPAWN_Y = 0 # 13.0

def create_pose(navigator, absolute_x, absolute_y, yaw_degrees):
    """Helper function to create a Nav2 goal pose using Absolute Rink Coordinates"""
    goal = PoseStamped()
    goal.header.frame_id = 'odom'
    goal.header.stamp = navigator.get_clock().now().to_msg()
    
    goal.pose.position.x = absolute_x - SPAWN_X
    goal.pose.position.y = absolute_y - SPAWN_Y
    
    # Convert degrees to radians, then to simple 2D quaternion
    yaw = math.radians(yaw_degrees)
    goal.pose.orientation.z = math.sin(yaw / 2.0)
    goal.pose.orientation.w = math.cos(yaw / 2.0)
    return goal

def main():
    rclpy.init()
    navigator = BasicNavigator()
    
    print("Waiting for Nav2 to boot...")
    navigator.waitUntilNav2Active(localizer='controller_server')
    print("Nav2 is online! Executing test waypoint.")

    # Now you can use pure, absolute IIHF coordinates!
    route = []
    
    # This will now actually go to the absolute top-right of the rink!
    # route.append(create_pose(navigator, absolute_x=5.0, absolute_y=0.0, yaw_degrees=0))
    # route.append(create_pose(navigator, absolute_x=7.5, absolute_y=-1.67, yaw_degrees=-22.5))
    # route.append(create_pose(navigator, absolute_x=10.0, absolute_y=-3.34, yaw_degrees=-45))
    # route.append(create_pose(navigator, absolute_x=12.5, absolute_y=-5.0, yaw_degrees=-66.6))
    # route.append(create_pose(navigator, absolute_x=15.0, absolute_y=-6.67, yaw_degrees=-90))
    # route.append(create_pose(navigator, absolute_x=12.5, absolute_y=-8.34, yaw_degrees=-112.5))
    # route.append(create_pose(navigator, absolute_x=10.0, absolute_y=-10.0, yaw_degrees=-135))
    # route.append(create_pose(navigator, absolute_x=7.5, absolute_y=-11.67, yaw_degrees=-157.5))
    # route.append(create_pose(navigator, absolute_x=5.0, absolute_y=-13.34, yaw_degrees=-180))
    # route.append(create_pose(navigator, absolute_x=0.0, absolute_y=-13.34, yaw_degrees=-180))

    # drawing a simple curve 
    # center of the circle
    center_x = 5.0
    center_y = -6.67
    radius = 6.67

    for i in range(7):
        # Calculate the angle for this specific point (from 90 degrees to -90 degrees)
        angle_rad = (math.pi / 2) - (i * (math.pi / 10))
        
        # Calculate the exact X and Y on the circle's edge
        arc_x = center_x + (radius * math.cos(angle_rad))
        arc_y = center_y + (radius * math.sin(angle_rad))
        
        # Calculate the robot's facing direction (tangent to the curve)
        yaw_rad = angle_rad - (math.pi / 2)
        
        route.append(create_pose(navigator, arc_x, arc_y, yaw_rad))



    print("Sending absolute coordinate target...")
    navigator.goThroughPoses(route)
    # navigator.followWaypoints(route)

    while not navigator.isTaskComplete():
        feedback = navigator.getFeedback()
        if feedback:
            print(f"Driving... Distance remaining: {feedback.distance_remaining:.2f} meters", end='\r')

    result = navigator.getResult()
    if result == TaskResult.SUCCEEDED:
        print("\nTarget Reached!")
    else:
        print(f"\nRoute failed or canceled. Result code: {result}")

    rclpy.shutdown()

if __name__ == '__main__':
    main()