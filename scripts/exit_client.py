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
        
    def start_exit_maneuver(self):
        self.get_logger().info('Phase 3: Waiting for NavigateToPose action server...')
        self._nav_to_pose_client.wait_for_server()
        
        self.get_logger().info('Server found. Generating dynamic route to Garage...')

        # --- GARAGE COORDINATES ---
        # Based on your early logs, the garage is at X: -32.0, Y: -10.75
        target_x = -35.0
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
        send_goal_future.add_done_callback(self._exit_goal_response_callback)

    def _exit_goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Exit goal rejected by Nav2! Check costmaps or tolerances.')
            return

        self.get_logger().info('Exit goal accepted. Zamboni is heading home...')
        get_result_future = goal_handle.get_result_async()
        get_result_future.add_done_callback(self._exit_result_callback)

    def _exit_result_callback(self, future):
        result = future.result()
        if result.status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('Mission Complete! Zamboni is safely parked in the garage.')
        else:
            self.get_logger().error(f'Exit maneuver failed with status code: {result.status}')

def main(args=None):
    rclpy.init(args=args)
    client = ZamboniExitNode()
    
    # Trigger the exit immediately on startup
    client.start_exit_maneuver()
    
    rclpy.spin(client)
    client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()