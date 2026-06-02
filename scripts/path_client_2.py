import math
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped, Quaternion
from nav_msgs.msg import Path
from nav2_msgs.action import FollowPath
from scipy.interpolate import splprep, splev

def get_zamboni_coordinates():
    

    def get_lap_points(offset=0.0):
      """
      Generates control points shifted inward by a specific offset distance.
      """
      # Top Left (Flowing right) - Push X right (+), Push Y down (-)
      top_left_x = [-28.0+offset, -28.0+offset, -28.0+offset, -27.5+offset, -26.0+offset, -23.0+offset, -20.0+offset, -10.0]
      top_left_y = [0.0,          5.0-offset,   8.0-offset,   10.0-offset,  12.0-offset,  13.0-offset,  13.0-offset,  13.0-offset]

      # Top Right - X pushes Left (-), Y pushes Down (-)
      top_right_x = [0.0,         10.0-offset,  20.0-offset,  23.0-offset,  26.0-offset,  27.5-offset,  28.0-offset,  28.0-offset]
      top_right_y = [13.0-offset, 13.0-offset,  13.0-offset,  13.0-offset,  12.0-offset,  10.0-offset,  8.0-offset,   5.0-offset]

      # Bottom Right - X pushes Left (-), Y pushes Up (+)
      bottom_right_x = [28.0-offset, 28.0-offset, 28.0-offset, 27.5-offset, 26.0-offset, 23.0-offset, 20.0-offset, 10.0]
      bottom_right_y = [0.0,         -5.0+offset, -8.0+offset, -10.0+offset, -12.0+offset, -13.0+offset, -13.0+offset, -13.0+offset]

      # Bottom Left - X pushes Right (+), Y pushes Up (+)
      bottom_left_x = [0.0,          -10.0,        -20.0+offset, -23.0+offset, -26.0+offset, -27.5+offset, -28.0+offset, -28.0+offset]
      bottom_left_y = [-13.0+offset, -13.0+offset, -13.0+offset, -13.0+offset, -12.0+offset, -10.0+offset, -8.0+offset,  -5.0]

      # Because you removed duplicate points, we can safely add them cleanly!
      cx = top_left_x + top_right_x + bottom_right_x + bottom_left_x
      cy = top_left_y + top_right_y + bottom_right_y + bottom_left_y

      # Close the loop
      # cx.append(cx[0])
      # cy.append(cy[0])
      return cx, cy

    # 1. Generate the raw control points for both laps using your exact math
    cx1, cy1 = get_lap_points(offset=0.0)
    cx2, cy2 = get_lap_points(offset=2.2)
    cx3, cy3 = get_lap_points(offset=4.4)

    # 2. Merge them into a single continuous sequence
    cx_total =  cx2 + cx3
    cy_total =  cy2 + cy3

    # 3. Generate the single spline trajectory
    # CRITICAL: We removed per=1 because this is a spiral, not a closed circle!
    tck, u = splprep([cx_total, cy_total], s=0, k=3)

    # We increase the linspace from 150 to 300 because the path is twice as long now.
    # This keeps the distance between the ROS 2 waypoints consistent.
    u_fine = np.linspace(0, 1, 300)
    smooth_x, smooth_y = splev(u_fine, tck)

    return smooth_x, smooth_y

# --- HELPER: Convert Yaw to Quaternion ---
def get_quaternion_from_yaw(yaw):
    q = Quaternion()
    q.x = 0.0
    q.y = 0.0
    q.z = math.sin(yaw / 2.0)
    q.w = math.cos(yaw / 2.0)
    return q

class ZamboniPathClient(Node):
    def __init__(self):
        super().__init__('zamboni_path_client')
        
        # Connect to the Nav2 FollowPath Action Server
        self._action_client = ActionClient(self, FollowPath, 'follow_path')

    def send_path(self):
        self.get_logger().info('Generating mathematical trajectory...')
        smooth_x, smooth_y = get_zamboni_coordinates()

        # Create the empty ROS 2 Path message
        path_msg = Path()
        path_msg.header.frame_id = 'map'
        path_msg.header.stamp = self.get_clock().now().to_msg()

        self.get_logger().info('Converting to ROS 2 Poses and calculating headings...')
        
        # Loop through all coordinates (except the last one, to calculate heading)
        for i in range(len(smooth_x) - 1):
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.header.stamp = path_msg.header.stamp
            
            # 1. Set Position
            pose.pose.position.x = float(smooth_x[i])
            pose.pose.position.y = float(smooth_y[i])
            pose.pose.position.z = 0.0

            # 2. Calculate Heading (atan2 of the delta between this point and the next)
            dy = smooth_y[i+1] - smooth_y[i]
            dx = smooth_x[i+1] - smooth_x[i]
            yaw = math.atan2(dy, dx)
            
            # 3. Set Orientation
            pose.pose.orientation = get_quaternion_from_yaw(yaw)
            
            path_msg.poses.append(pose)

        # Handle the very last point (just copy the heading from the previous point)
        final_pose = PoseStamped()
        final_pose.header = path_msg.poses[-1].header
        final_pose.pose.position.x = float(smooth_x[-1])
        final_pose.pose.position.y = float(smooth_y[-1])
        final_pose.pose.orientation = path_msg.poses[-1].pose.orientation
        path_msg.poses.append(final_pose)

        # Wait for Nav2 to be ready
        self.get_logger().info('Waiting for FollowPath action server...')
        self._action_client.wait_for_server()

        # Send the goal!
        goal_msg = FollowPath.Goal()
        goal_msg.path = path_msg
        # You can specify a specific controller here if you have multiple in nav2_params
        # goal_msg.controller_id = 'FollowPath' 

        self.get_logger().info('Sending massive Zamboni trajectory to Nav2!')
        self._action_client.send_goal_async(goal_msg)

def main(args=None):
    rclpy.init(args=args)
    client = ZamboniPathClient()
    client.send_path()
    rclpy.spin(client)

if __name__ == '__main__':
    main()