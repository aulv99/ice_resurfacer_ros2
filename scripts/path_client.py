import math
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped, Quaternion
from nav_msgs.msg import Path
from nav2_msgs.action import FollowPath
from scipy.interpolate import splprep, splev


# --- PASTE YOUR EXACT GET_LAP_POINTS AND SPLPREP MATH HERE ---
# Instead of plotting it, just have the function return smooth_x, smooth_y
def get_zamboni_coordinates():
    # ... (Your math) ...
    # return smooth_x, smooth_y

    # def generate_zamboni_path():

    def get_lap_points(offset=0.0):
        # Top Left
        top_left_x = [
            -28.835 + offset,
            -28.835 + offset,
            -28.25 + (offset * 0.95),
            -27.35 + (offset * 0.80),
            -25.35 + (offset * 0.60),
            -22.45 + (offset * 0.15),
            -20.0,
            -10.0
        ]
        top_left_y = [
            0.0,
            6.0,
            8.7 - (offset * 0.15),
            10.6 - (offset * 0.60),
            12.8 - (offset * 0.80),
            13.635 - (offset * 0.95),
            13.785 - offset,
            13.785 - offset
        ]

        # Top Right
        top_right_x = [
            0.0,
            10.0,
            20.0,
            22.9 - (offset * 0.15),
            25.85 - (offset * 0.60),
            27.9 - (offset * 0.80),
            28.55 - (offset * 0.95),
            28.835 - offset
        ]
        top_right_y = [
            13.835 - offset,
            13.835 - offset,
            13.835 - offset,
            13.735 - (offset * 0.95),
            12.85 - (offset * 0.80),
            10.9 - (offset * 0.60),
            9.0 - (offset * 0.15),
            6.0
        ]

        # Bottom Right
        bottom_right_x = [
            28.835 - offset,
            28.835 - offset,
            28.585 - (offset * 0.95),
            27.85 - (offset * 0.80),
            25.75 - (offset * 0.60),
            22.75 - (offset * 0.15),
            20.0,
            10.0
        ]
        bottom_right_y = [
            0.0,
            -5.75,
            -8.75 + (offset * 0.15),
            -10.75 + (offset * 0.60),
            -13.0 + (offset * 0.80),
            -13.685 + (offset * 0.95),
            -13.835 + offset,
            -13.835 + offset
        ]

        # Bottom Left
        bottom_left_x = [
            0.0,
            -10.0,
            -20.0,
            -22.9 + (offset * 0.15),
            -25.9 + (offset * 0.60),
            -27.9 + (offset * 0.80),
            -28.65 + (offset * 0.95),
            -28.835 + offset
        ]
        bottom_left_y = [
            -13.835 + offset,
            -13.835 + offset,
            -13.835 + offset,
            -13.65 + (offset * 0.95),
            -12.9 + (offset * 0.80),
            -10.9 + (offset * 0.60),
            -8.9 + (offset * 0.15),
            -6.0
        ]

        cx = top_left_x + top_right_x + bottom_right_x + bottom_left_x
        cy = top_left_y + top_right_y + bottom_right_y + bottom_left_y

        return cx, cy

    def get_shifting_sweep_points(offset=0.0, is_last_lap=False, is_first_lap=False):
        """
        Generates the inner closed loop.
        Offset shifts the ENTIRE loop UPWARDS.
        If is_last_lap is True, omits the final left turn.
        """

        if is_first_lap:
            # The very first lap needs to start deep in the center
            pass1_x = [-25.0, -20.0, -10.0, 0.0, 10.0, 20.0]
            pass1_y = [0.75 + offset, 1.0 + offset, 1.0 + offset, 1.0 + offset, 1.0 + offset, 1.0 + offset]
        else:
            # All normal laps start further right to avoid crowding the left boards
            pass1_x = [-20.0, -10.0, 0.0, 10.0, 20.0]
            pass1_y = [1.0 + offset, 1.0 + offset, 1.0 + offset, 1.0 + offset, 1.0 + offset]

        # RIGHT TURN
        right_turn_x = [25.5, 26.5, 25.5]
        right_turn_y = [-0.5 + offset, -5.0 + offset, -9.0 + offset]

        if is_last_lap:
            # Stretch the final return trip all the way to X=-25.0 to close the gap
            pass2_x = [20.0, 10.0, 0.0, -10.0, -20.0, -25.0]
            pass2_y = [-10.4 + offset, -10.4 + offset, -10.4 + offset, -10.4 + offset, -10.4 + offset, -10.4 + offset]
        else:
            # Normal return trips stop at -20.0 to prepare for the left turn
            pass2_x = [20.0, 10.0, 0.0, -10.0, -20.0]
            pass2_y = [-10.4 + offset, -10.4 + offset, -10.4 + offset, -10.4 + offset, -10.4 + offset]

        # LEFT TURN
        left_turn_x = [-25.5, -26.5, -25.5]
        left_turn_y = [-9.0 + offset, -3.0 + offset, 2.3 + offset]

        # STITCHING LOGIC
        if is_last_lap:
            # Drop the left turn completely! The path just ends at X=-20.0 or -25.0
            cx = pass1_x + right_turn_x + pass2_x
            cy = pass1_y + right_turn_y + pass2_y
        else:
            # Build the normal continuous loop
            cx = pass1_x + right_turn_x + pass2_x + left_turn_x
            cy = pass1_y + right_turn_y + pass2_y + left_turn_y
            
        return cx, cy

    # 1. Generate the raw control points for the outer laps
    cx1, cy1 = get_lap_points(offset=0.0)
    cx2, cy2 = get_lap_points(offset=2.0)

    # 2. Generate the center sweeps
    cx_sweep1, cy_sweep1 = get_shifting_sweep_points(offset=0.0, is_last_lap=False, is_first_lap=True)
    cx_sweep2, cy_sweep2 = get_shifting_sweep_points(offset=2.0, is_last_lap=False, is_first_lap=False)
    cx_sweep3, cy_sweep3 = get_shifting_sweep_points(offset=4.0, is_last_lap=False, is_first_lap=False)
    cx_sweep4, cy_sweep4 = get_shifting_sweep_points(offset=6.0, is_last_lap=False, is_first_lap=False)
    cx_sweep5, cy_sweep5 = get_shifting_sweep_points(offset=8.0, is_last_lap=False, is_first_lap=False)
    cx_sweep6, cy_sweep6 = get_shifting_sweep_points(offset=10.0, is_last_lap=True, is_first_lap=False)
    
    # 3. STITCH THEM ALL TOGETHER
    cx_total = cx1 + cx2 + cx_sweep1 + cx_sweep2 + cx_sweep3 + cx_sweep4 + cx_sweep5 + cx_sweep6 
    cy_total = cy1 + cy2 + cy_sweep1 + cy_sweep2 + cy_sweep3 + cy_sweep4 + cy_sweep5 + cy_sweep6 

    # 4. Generate the single spline trajectory
    tck, u = splprep([cx_total, cy_total], s=0, k=2)

    u_fine = np.linspace(0, 1, 750)
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