import math
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import Quaternion, Twist
from nav_msgs.msg import Odometry
from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus
from rclpy.qos import qos_profile_sensor_data

def get_quaternion_from_yaw(yaw):
    q = Quaternion()
    q.x = 0.0
    q.y = 0.0
    q.z = math.sin(yaw / 2.0)
    q.w = math.cos(yaw / 2.0)
    return q

class ZamboniExitNode(Node):
    def __init__(self):
        super().__init__('zamboni_exit_node')
        
        # --- Action Client & Publishers/Subscribers ---
        self._nav_to_pose_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.odom_sub = self.create_subscription(
            Odometry, 
            '/odometry/filtered', 
            self.odom_callback, 
            qos_profile_sensor_data
        )

        # State tracker to manage the manual reverse
        self.mission_state = 'NAVIGATING'
        
    # --- ODOMETRY CALLBACK (MANUAL OVERRIDE) ---
    def odom_callback(self, msg):
        if self.mission_state == 'REVERSING_OUT_OF_PIT':
            
            # 1. Extract Zamboni's current Yaw angle from the Quaternion
            q = msg.pose.pose.orientation
            siny_cosp = 2 * (q.w * q.z + q.x * q.y)
            cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
            current_yaw = math.atan2(siny_cosp, cosy_cosp)
            
            # 2. Check if we have reached 0.0 (East) with a small tolerance
            # We start at -1.57 (South). We keep turning while we are less than -0.05
            if current_yaw < -0.05:
                twist = Twist()
                twist.linear.x = -0.5  # Reverse at 0.5 m/s
                
                # Note: 5.0 rad/s is a massive steering command for Ackermann. 
                # Your controller will likely just cap this at maximum steering lock.
                twist.angular.z = 0.1  
                
                self.cmd_vel_pub.publish(twist)
            else:
                # We hit the target angle! Hit the brakes.
                self.cmd_vel_pub.publish(Twist())
                self.mission_state = 'NAVIGATING'
                self.get_logger().info(f'Angle achieved! (Yaw: {current_yaw:.2f}). Handing back to Nav2...')
                self.start_exit_maneuver_3c()

    # --- NAVIGATE OUT OF RINK (PHASE 3A) ---
    def start_exit_maneuver_3a(self):
        self.get_logger().info('Phase 3A: Waiting for NavigateToPose action server...')
        self._nav_to_pose_client.wait_for_server()
        
        self.get_logger().info('Server found. Generating dynamic route to Garage...')

        target_x = -34.5
        target_y = -10.25
        target_yaw = math.pi  

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        
        goal_msg.pose.pose.position.x = float(target_x)
        goal_msg.pose.pose.position.y = float(target_y)
        goal_msg.pose.pose.position.z = 0.0
        goal_msg.pose.pose.orientation = get_quaternion_from_yaw(target_yaw)

        self.get_logger().info(f'Sending Exit Goal -> X: {target_x:.2f}, Y: {target_y:.2f}')
        
        send_goal_future = self._nav_to_pose_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self._exit_goal_3a_response_callback)

    def _exit_goal_3a_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Exit goal rejected by Nav2!')
            return

        self.get_logger().info('Exit goal accepted. Zamboni is heading home...')
        get_result_future = goal_handle.get_result_async()
        get_result_future.add_done_callback(self._exit_result_3a_callback)

    def _exit_result_3a_callback(self, future):
        result = future.result()
        if result.status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('Phase 3A Complete! Zamboni is safely at the garage tunnel.')
            self.start_exit_maneuver_3b()
        else:
            self.get_logger().error(f'Exit maneuver failed with status code: {result.status}')

    # --- NAVIGATE TO SNOW UNLOAD STATION (PHASE 3B) ---
    def start_exit_maneuver_3b(self):
        self.get_logger().info('Phase 3B: Transitioning to Snow Unload Station...')

        target_x = -41.50
        target_y = -20.0
        target_yaw = -math.pi / 2

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        
        goal_msg.pose.pose.position.x = float(target_x)
        goal_msg.pose.pose.position.y = float(target_y)
        goal_msg.pose.pose.position.z = 0.0
        goal_msg.pose.pose.orientation = get_quaternion_from_yaw(target_yaw)

        self.get_logger().info(f'Sending Exit Goal -> X: {target_x:.2f}, Y: {target_y:.2f}')
        
        send_goal_future = self._nav_to_pose_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self._exit_goal_3b_response_callback)

    def _exit_goal_3b_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Exit goal rejected by Nav2!')
            return

        self.get_logger().info('Exit goal accepted. Zamboni is heading out...')
        get_result_future = goal_handle.get_result_async()
        get_result_future.add_done_callback(self._exit_result_3b_callback)

    def _exit_result_3b_callback(self, future):
        result = future.result()
        if result.status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('Unloading Snow. Waiting 3 seconds for the pit to clear...')
            
            # Publish a zero-velocity Twist to ensure the brakes are held during the wait
            self.cmd_vel_pub.publish(Twist())
            
            # Create a one-shot timer that waits 3.0 seconds, then calls the reverse function
            self.delay_timer = self.create_timer(3.0, self._trigger_manual_reverse)
        else:
            self.get_logger().error(f'Exit maneuver failed with status code: {result.status}')

    def _trigger_manual_reverse(self):
        # 1. Destroy the timer so it only executes exactly once
        self.delay_timer.cancel()
        
        # 2. Change the state to unlock the odom_callback
        self.get_logger().info('Wait complete. Engaging Manual Reverse Override...')
        self.mission_state = 'REVERSING_OUT_OF_PIT'

    # --- PARK TO GARAGE (PHASE 3C) ---
    def start_exit_maneuver_3c(self):
        self.get_logger().info('Phase 3C: Parking to Garage...')

        target_x = -34.5
        target_y = -10.25
        target_yaw = 0.0

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        
        goal_msg.pose.pose.position.x = float(target_x)
        goal_msg.pose.pose.position.y = float(target_y)
        goal_msg.pose.pose.position.z = 0.0
        goal_msg.pose.pose.orientation = get_quaternion_from_yaw(target_yaw)

        self.get_logger().info(f'Sending Final Parking Goal -> X: {target_x:.2f}, Y: {target_y:.2f}')
        
        send_goal_future = self._nav_to_pose_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self._exit_goal_3c_response_callback)

    def _exit_goal_3c_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Parking goal rejected by Nav2!')
            return

        self.get_logger().info('Parking goal accepted. Completing final maneuver...')
        get_result_future = goal_handle.get_result_async()
        get_result_future.add_done_callback(self._exit_result_3c_callback)

    def _exit_result_3c_callback(self, future):
        result = future.result()
        if result.status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('Mission Complete! Zamboni is safely parked.')
        else:
            self.get_logger().error(f'Parking maneuver failed with status code: {result.status}')

def main(args=None):
    rclpy.init(args=args)
    client = ZamboniExitNode()
    
    # Trigger the exit immediately on startup
    client.start_exit_maneuver_3a()
    
    rclpy.spin(client)
    client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()