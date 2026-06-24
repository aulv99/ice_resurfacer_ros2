import math
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import Quaternion
from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus

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
        
        # Action Client for Nav2's global planner
        self._nav_to_pose_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        
    # --- NAVIGATE OUT OF RINK (PHASE 3A) ---
    
    def start_exit_maneuver_3a(self):
        self.get_logger().info('Phase 3A: Waiting for NavigateToPose action server...')
        self._nav_to_pose_client.wait_for_server()
        
        self.get_logger().info('Server found. Generating dynamic route to Garage...')

        # --- GARAGE COORDINATES ---
        # Based on your early logs, the garage is at X: -32.0, Y: -10.75
        target_x = -34.5
        target_y = -10.25
        
        # You can adjust this based on how you want it parked in the garage.
        # math.pi = Facing West (pulling straight in). 
        # 0.0 = Facing East (backed in). Nav2's Reeds-Shepp will reverse it automatically if needed!
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
            self.get_logger().error('Exit goal rejected by Nav2! Check costmaps or tolerances.')
            return

        self.get_logger().info('Exit goal accepted. Zamboni is heading home...')
        get_result_future = goal_handle.get_result_async()
        get_result_future.add_done_callback(self._exit_result_3a_callback)

    def _exit_result_3a_callback(self, future):
        result = future.result()
        if result.status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('Mission Complete! Zamboni is safely in the garage.')
            self.start_exit_maneuver_3b()
        else:
            self.get_logger().error(f'Exit maneuver failed with status code: {result.status}')

    # --- NAVIGATE TO SNOW UNLOAD STATION (PHASE 3B) ---
    def start_exit_maneuver_3b(self):
        self.get_logger().info('Phase 3B: Transitioning to Snow Unload Station...')

        # Snow Unloading station coordinates
        target_x = -42.50
        target_y = -15
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
            self.get_logger().error('Exit goal rejected by Nav2! Check costmaps or tolerances.')
            return

        self.get_logger().info('Exit goal accepted. Zamboni is heading out...')
        get_result_future = goal_handle.get_result_async()
        get_result_future.add_done_callback(self._exit_result_3b_callback)

    def _exit_result_3b_callback(self, future):
        result = future.result()
        if result.status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('Mission Complete! Zamboni is unloading snow storage')
            self.start_exit_maneuver_3c()
        else:
            self.get_logger().error(f'Exit maneuver failed with status code: {result.status}')

    # --- PARK TO GARAGE (PHASE 3C) ---
    def start_exit_maneuver_3c(self):
        self.get_logger().info('Phase 3C: Parking to Garage...')

        # Snow Unloading station coordinates
        target_x = -46
        target_y = -10.25
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
        send_goal_future.add_done_callback(self._exit_goal_3c_response_callback)

    def _exit_goal_3c_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Exit goal rejected by Nav2! Check costmaps or tolerances.')
            return

        self.get_logger().info('Exit goal accepted. Zamboni is heading home...')
        get_result_future = goal_handle.get_result_async()
        get_result_future.add_done_callback(self._exit_result_3c_callback)

    def _exit_result_3c_callback(self, future):
        result = future.result()
        if result.status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('Mission Complete! Zamboni is safely parked in the garage.')
        else:
            self.get_logger().error(f'Exit maneuver failed with status code: {result.status}')

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