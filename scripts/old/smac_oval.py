#!/usr/bin/env python3
import rclpy
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from geometry_msgs.msg import PoseStamped
import math

def main():
    rclpy.init()
    navigator = BasicNavigator()

    print("Waiting for Nav2...")
    navigator.waitUntilNav2Active(localizer='bt_navigator')
    
    print("Executing...")

    # Create a single target directly in front of the robot
    goal_pose = PoseStamped()
    goal_pose.header.frame_id = 'map'
    goal_pose.header.stamp = navigator.get_clock().now().to_msg()
    
    # 8 meters forward, 0 meters left/right
    goal_pose.pose.position.x = 8.0
    goal_pose.pose.position.y = 0.0
    goal_pose.pose.position.z = 0.0
    
    # Facing perfectly straight (0 radians)
    goal_pose.pose.orientation.x = 0.0
    goal_pose.pose.orientation.y = 0.0
    goal_pose.pose.orientation.z = 0.0
    goal_pose.pose.orientation.w = 1.0

    # Notice we are using goToPose, NOT goThroughPoses
    navigator.goToPose(goal_pose)

    while not navigator.isTaskComplete():
        pass 

    result = navigator.getResult()
    if result == TaskResult.SUCCEEDED:
        print("Success!")
    else:
        print("Failed.")

    navigator.lifecycleShutdown()
    rclpy.shutdown()

if __name__ == '__main__':
    main()