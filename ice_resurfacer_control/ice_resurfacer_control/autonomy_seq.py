import math
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped, Quaternion, Twist
from nav_msgs.msg import Path, Odometry
from nav2_msgs.action import FollowPath, NavigateToPose
from nav2_msgs.msg import SpeedLimit
from action_msgs.msg import GoalStatus
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy, qos_profile_sensor_data
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from std_msgs.msg import String
from std_srvs.srv import Trigger

# ============================================================
# RINK & ZAMBONI PARAMETERS
# ============================================================
RINK_LENGTH = 60.0
RINK_WIDTH = 30.0
CORNER_RADIUS = 8.5
CONDITIONER_WIDTH = 2.13
LANE_SPACING = 1.9          
SAFETY_MARGIN = 0.10
POINT_SPACING = 0.10

# --- THE SPEED PROFILES ---
WALL_STRAIGHT_SPEED = 0.5 # 2.0
WALL_CORNER_SPEED = 0.5 # 1.0
SWEEP_STRAIGHT_SPEED = 0.5 # 2.0
SWEEP_CORNER_SPEED = 0.5 # 1.5

# ============================================================
# GEOMETRY HELPERS 
# ============================================================
def sample_line(p1, p2, spacing=POINT_SPACING):
    x1, y1 = p1
    x2, y2 = p2
    length = math.hypot(x2 - x1, y2 - y1)
    if length == 0:
        return [x1], [y1]
    n = max(2, int(length / spacing))
    xs = np.linspace(x1, x2, n)
    ys = np.linspace(y1, y2, n)
    return xs.tolist(), ys.tolist()

def sample_arc(center, radius, start_angle, end_angle, spacing=POINT_SPACING):
    arc_len = radius * abs(end_angle - start_angle)
    if arc_len == 0:
        return [center[0] + radius * math.cos(start_angle)], [center[1] + radius * math.sin(start_angle)]
    n = max(5, int(arc_len / spacing))
    angles = np.linspace(start_angle, end_angle, n)
    xs = center[0] + radius * np.cos(angles)
    ys = center[1] + radius * np.sin(angles)
    return xs.tolist(), ys.tolist()

def calculate_dynamic_coord(val, val_start, val_end, base_coord, offset_s, offset_c, sign):
    mid_val = (val_start + val_end) / 2.0
    max_dist = abs(val_end - mid_val)
    if max_dist == 0: return base_coord + sign * offset_s
    
    ratio = abs(val - mid_val) / max_dist
    current_offset = offset_s + (offset_c - offset_s) * ratio
    return base_coord + sign * current_offset

def sample_dynamic_line_y_major(y1, y2, y_full_start, y_full_end, base_x, offset_s, offset_c, sign, spacing=POINT_SPACING):
    length = abs(y2 - y1)
    if length == 0:
        x = calculate_dynamic_coord(y1, y_full_start, y_full_end, base_x, offset_s, offset_c, sign)
        return [x], [y1]
    n = max(2, int(length / spacing))
    ys = np.linspace(y1, y2, n)
    xs = [calculate_dynamic_coord(y, y_full_start, y_full_end, base_x, offset_s, offset_c, sign) for y in ys]
    return xs, ys.tolist()

def sample_dynamic_line_x_major(x1, x2, x_full_start, x_full_end, base_y, offset_s, offset_c, sign, spacing=POINT_SPACING):
    length = abs(x2 - x1)
    if length == 0:
        y = calculate_dynamic_coord(x1, x_full_start, x_full_end, base_y, offset_s, offset_c, sign)
        return [x1], [y]
    n = max(2, int(length / spacing))
    xs = np.linspace(x1, x2, n)
    ys = [calculate_dynamic_coord(x, x_full_start, x_full_end, base_y, offset_s, offset_c, sign) for x in xs]
    return xs.tolist(), ys

def append_segment(path_x, path_y, path_v, seg_x, seg_y, target_speed):
    if len(path_x) > 0 and len(seg_x) > 0:
        seg_x = seg_x[1:]
        seg_y = seg_y[1:]
    path_x.extend(seg_x)
    path_y.extend(seg_y)
    path_v.extend([target_speed] * len(seg_x))

def generate_wall_lane_change_horizontal(start_x, end_x, start_y, end_y, n_points=40):
    local_x = np.linspace(0.0, end_x - start_x, n_points)
    progress = np.linspace(0, math.pi, n_points)
    
    y_shift = end_y - start_y
    local_y = (y_shift / 2.0) * (1 - np.cos(progress))
    
    x = start_x + local_x
    y = start_y + local_y
    return x.tolist(), y.tolist()

def generate_wall_to_sweep_transition(x_wall, y_sweep_top, x_sweep_start):
    x = []
    y = []
    v = []
    radius = abs(x_sweep_start - x_wall)
    y_wall_end = y_sweep_top - radius
    center_x = x_sweep_start
    center_y = y_wall_end
    
    sx, sy = sample_arc((center_x, center_y), radius, math.pi, math.pi/2)
    append_segment(x, y, v, sx, sy, WALL_CORNER_SPEED)
    
    return x, y, v, y_wall_end

def generate_staging_alignment(staging_x, staging_y, target_x, target_y):
    x, y, v = [], [], []
    u_turn_start_x = -10.0
    if staging_x < u_turn_start_x:
        sx, sy = sample_line((staging_x, staging_y), (u_turn_start_x, staging_y))
        append_segment(x, y, v, sx, sy, WALL_STRAIGHT_SPEED)
        
    u_turn_end_y = -13.0
    radius = abs(staging_y - u_turn_end_y) / 2.0
    center_x = u_turn_start_x
    center_y = (staging_y + u_turn_end_y) / 2.0
    
    sx, sy = sample_arc((center_x, center_y), radius, math.pi/2, -math.pi/2)
    append_segment(x, y, v, sx, sy, WALL_CORNER_SPEED)
    
    tx, ty = generate_wall_lane_change_horizontal(
        start_x=u_turn_start_x, end_x=target_x, start_y=u_turn_end_y, end_y=target_y, n_points=60
    )
    append_segment(x, y, v, tx, ty, WALL_STRAIGHT_SPEED)
    
    return x, y, v

def generate_classic_zamboni_sweeps(x_limit, y_top_start, y_bottom_start, lane_spacing, num_passes):
    x, y, v = [], [], []
    current_y_top = y_top_start
    current_y_bottom = y_bottom_start

    for i in range(num_passes):
        sx, sy = sample_line((-x_limit, current_y_top), (x_limit, current_y_top))
        append_segment(x, y, v, sx, sy, SWEEP_STRAIGHT_SPEED)

        radius_east = (current_y_top - current_y_bottom) / 2.0
        center_y_east = (current_y_top + current_y_bottom) / 2.0
        sx, sy = sample_arc((x_limit, center_y_east), radius_east, math.pi/2, -math.pi/2)
        append_segment(x, y, v, sx, sy, SWEEP_CORNER_SPEED)

        sx, sy = sample_line((x_limit, current_y_bottom), (-x_limit, current_y_bottom))
        append_segment(x, y, v, sx, sy, SWEEP_STRAIGHT_SPEED)

        if i == num_passes - 1:
            break

        next_y_top = current_y_top + lane_spacing
        radius_west = (next_y_top - current_y_bottom) / 2.0
        center_y_west = (next_y_top + current_y_bottom) / 2.0
        sx, sy = sample_arc((-x_limit, center_y_west), radius_west, -math.pi/2, -3*math.pi/2)
        append_segment(x, y, v, sx, sy, SWEEP_CORNER_SPEED)

        current_y_top = next_y_top
        current_y_bottom += lane_spacing

    return x, y, v

def generate_full_wall_lap_bottom(offset_straight, offset_corner, start_x, end_x):
    x, y, v = [], [], []
    r = CORNER_RADIUS - offset_corner

    cx_tl, cy_tl = -30 + CORNER_RADIUS, 15 - CORNER_RADIUS
    cx_tr, cy_tr = 30 - CORNER_RADIUS, 15 - CORNER_RADIUS
    cx_br, cy_br = 30 - CORNER_RADIUS, -15 + CORNER_RADIUS
    cx_bl, cy_bl = -30 + CORNER_RADIUS, -15 + CORNER_RADIUS

    sx, sy = sample_dynamic_line_x_major(start_x, cx_bl, cx_br, cx_bl, -15.0, offset_straight, offset_corner, +1)
    append_segment(x, y, v, sx, sy, WALL_STRAIGHT_SPEED)
    sx, sy = sample_arc((cx_bl, cy_bl), r, -math.pi/2, -math.pi)
    append_segment(x, y, v, sx, sy, WALL_CORNER_SPEED)
    sx, sy = sample_dynamic_line_y_major(cy_bl, cy_tl, cy_bl, cy_tl, -30.0, offset_straight, offset_corner, +1)
    append_segment(x, y, v, sx, sy, WALL_STRAIGHT_SPEED)
    sx, sy = sample_arc((cx_tl, cy_tl), r, math.pi, math.pi/2)
    append_segment(x, y, v, sx, sy, WALL_CORNER_SPEED)
    sx, sy = sample_dynamic_line_x_major(cx_tl, cx_tr, cx_tl, cx_tr, 15.0, offset_straight, offset_corner, -1)
    append_segment(x, y, v, sx, sy, WALL_STRAIGHT_SPEED)
    sx, sy = sample_arc((cx_tr, cy_tr), r, math.pi/2, 0)
    append_segment(x, y, v, sx, sy, WALL_CORNER_SPEED)
    sx, sy = sample_dynamic_line_y_major(cy_tr, cy_br, cy_tr, cy_br, 30.0, offset_straight, offset_corner, -1)
    append_segment(x, y, v, sx, sy, WALL_STRAIGHT_SPEED)
    sx, sy = sample_arc((cx_br, cy_br), r, 0, -math.pi/2)
    append_segment(x, y, v, sx, sy, WALL_CORNER_SPEED)
    sx, sy = sample_dynamic_line_x_major(cx_br, end_x, cx_br, cx_bl, -15.0, offset_straight, offset_corner, +1)
    append_segment(x, y, v, sx, sy, WALL_STRAIGHT_SPEED)

    return x, y, v

def generate_zamboni_path():
    px, py, pv = [], [], []

    margin_straight = 0.03
    margin_corner = 0.06
    first_offset_straight = CONDITIONER_WIDTH/2 + margin_straight
    first_offset_corner = CONDITIONER_WIDTH/2 + margin_corner

    cx_br = 30.0 - CORNER_RADIUS
    cx_bl = -30.0 + CORNER_RADIUS
    cy_tl = 15.0 - CORNER_RADIUS
    cy_bl = -15.0 + CORNER_RADIUS

    lap0_start_x = -15.0
    lap0_end_x = -15.0
    lap0_start_y = -15.0 + first_offset_straight

    staging_x = -15.0
    staging_y = -2.0
    tx0, ty0, tv0 = generate_staging_alignment(
        staging_x=staging_x, staging_y=staging_y, target_x=lap0_start_x, target_y=lap0_start_y
    )
    px.extend(tx0); py.extend(ty0); pv.extend(tv0)

    lx0, ly0, lv0 = generate_full_wall_lap_bottom(
        offset_straight=first_offset_straight, offset_corner=first_offset_corner, start_x=lap0_start_x, end_x=lap0_end_x
    )
    px.extend(lx0); py.extend(ly0); pv.extend(lv0)

    lap0_end_y = calculate_dynamic_coord(lap0_end_x, cx_br, cx_bl, -15.0, first_offset_straight, first_offset_corner, +1)
    
    offset_1 = first_offset_straight + LANE_SPACING 
    lap1_start_x = -20.0 
    lap1_start_y = -15.0 + offset_1 

    tx1, ty1 = generate_wall_lane_change_horizontal(
        start_x=lap0_end_x, end_x=lap1_start_x, start_y=lap0_end_y, end_y=lap1_start_y
    )
    append_segment(px, py, pv, tx1, ty1, WALL_CORNER_SPEED)

    lx1, ly1, lv1 = generate_full_wall_lap_bottom(
        offset_straight=offset_1, offset_corner=offset_1, start_x=lap1_start_x, end_x=lap1_start_x
    )
    px.extend(lx1); py.extend(ly1); pv.extend(lv1)

    lap1_left_x = -30.0 + offset_1  
    y_sweep_top_start = 1.0          
    x_sweep_start = -20.0            
    sweep_x_limit = 20.0             
    
    tx2, ty2, tv2, lap1_end_y = generate_wall_to_sweep_transition(
        x_wall=lap1_left_x, y_sweep_top=y_sweep_top_start, x_sweep_start=x_sweep_start
    )

    r1 = CORNER_RADIUS - offset_1
    
    sx, sy = sample_dynamic_line_x_major(lap1_start_x, cx_bl, cx_br, cx_bl, -15.0, offset_1, offset_1, +1)
    append_segment(px, py, pv, sx, sy, WALL_STRAIGHT_SPEED)

    sx, sy = sample_arc((cx_bl, cy_bl), r1, -math.pi/2, -math.pi)
    append_segment(px, py, pv, sx, sy, WALL_CORNER_SPEED)

    sx, sy = sample_dynamic_line_y_major(cy_bl, lap1_end_y, cy_bl, cy_tl, -30.0, offset_1, offset_1, +1)
    append_segment(px, py, pv, sx, sy, WALL_STRAIGHT_SPEED)

    px.extend(tx2); py.extend(ty2); pv.extend(tv2)

    sweep_x, sweep_y, sweep_v = generate_classic_zamboni_sweeps(
        x_limit=sweep_x_limit, y_top_start=y_sweep_top_start, y_bottom_start=-10.4, lane_spacing=LANE_SPACING, num_passes=6            
    )
    px.extend(sweep_x); py.extend(sweep_y); pv.extend(sweep_v)

    exit_radius = 5.0
    exit_center_x = px[-1]
    exit_center_y = py[-1] - exit_radius
    
    sx, sy = sample_arc((exit_center_x, exit_center_y), exit_radius, math.pi/2, math.pi)
    append_segment(px, py, pv, sx, sy, WALL_CORNER_SPEED)

    return px, py, pv

def get_quaternion_from_yaw(yaw):
    q = Quaternion()
    q.x = 0.0
    q.y = 0.0
    q.z = math.sin(yaw / 2.0)
    q.w = math.cos(yaw / 2.0)
    return q

# ============================================================
# MASTER ROS 2 CLIENT NODE (ALL PHASES)
# ============================================================
class ZamboniMasterNode(Node):
    def __init__(self):
        super().__init__('zamboni_master_node')
        
        # --- Action Clients & Publishers ---
        self._nav_to_pose_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self._nav_to_pose_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        # self._conditioner_client = ActionClient(self, FollowJointTrajectory, '/conditioner_controller/follow_joint_trajectory')
        self.state_pub = self.create_publisher(String, '/mission_state', 10)

        # ROS2 Service Server
        self.start_srv = self.create_service(Trigger, '/start_sequence', self.start_callback)
        self.stop_srv = self.create_service(Trigger, '/stop_sequence', self.stop_callback)
        
        # We need a variable to track if we are allowed to run
        self.is_running = False
        
        # --- State Tracking ---
        self.mission_state = 'IDLE'  
        self.current_path_index = 0
        self.path_x = []
        self.path_y = []
        self.path_v = []
        
        self.odom_sub = self.create_subscription(
            Odometry, 
            '/odometry/filtered', 
            self.odom_callback, 
            qos_profile_sensor_data
        )

    # 3. Create the Callbacks
    def start_callback(self, request, response):
        """ Triggered when the UI presses START """
        if self.is_running:
            response.success = False
            response.message = "Jäänajo on jo käynnissä"
            return response
            
        # Add safety checks here
        
        self.is_running = True
        self.get_logger().info("Jäänajo aloitettu ohjauspaneelista")
        
        # Triggering the 
        self.start_phase_1_transit()
        
        response.success = True
        response.message = "Sequence started successfully."
        return response

    def stop_callback(self, request, response):
        """ Triggered when the UI presses STOP """
        if not self.is_running:
            response.success = False
            response.message = "Already stopped."
            return response

        self.is_running = False
        self.get_logger().warn("Jäänajo keskeytetty")
        
        # -> INSERT YOUR CODE HERE to cancel the Nav2 goal and publish 0.0 to cmd_vel
        
        response.success = True
        response.message = "Emergency Stop executed."
        return response

    def set_and_publish_state(self, new_state):
        """ Updates internal state and broadcasts it to the ROS network """
        self.mission_state = new_state
        msg = String()
        msg.data = new_state
        self.state_pub.publish(msg)
        self.get_logger().info(f"Tila vaihtunut: {new_state} ")  

    # ============================================================
    # SHARED ODOMETRY CALLBACK (Traffic Controller)
    # ============================================================
    def odom_callback(self, msg):
        # ----------------------------------------------------
        # PHASE 2: RESURFACING PURE PURSUIT
        # ----------------------------------------------------
        if self.mission_state == 'RESURFACING': 
            rx = msg.pose.pose.position.x
            ry = msg.pose.pose.position.y
            
            q = msg.pose.pose.orientation
            siny_cosp = 2 * (q.w * q.z + q.x * q.y)
            cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
            ryaw = math.atan2(siny_cosp, cosy_cosp)

            search_window = 100 
            start_idx = self.current_path_index
            end_idx = min(start_idx + search_window, len(self.path_x))
            
            dx_arr = self.path_x[start_idx:end_idx] - rx
            dy_arr = self.path_y[start_idx:end_idx] - ry
            distances = dx_arr**2 + dy_arr**2
            local_closest = np.argmin(distances)
            
            self.current_path_index = start_idx + local_closest

            # Phase 2 Complete! Transition to Phase 3.
            if self.current_path_index >= len(self.path_x) - 5:
                self.cmd_vel_pub.publish(Twist())
                # self.set_conditioner_position(-0.2) 
                self.set_and_publish_state('TRANSITING_EXIT')
                self.get_logger().info('Jäänajo valmis. Ajetaan pois jäältä')
                self.start_exit_maneuver_3a()
                return

            lookahead_dist = 2.5  
            lookahead_idx = self.current_path_index
            
            while lookahead_idx < len(self.path_x) - 1:
                dist = math.hypot(self.path_x[lookahead_idx] - rx, self.path_y[lookahead_idx] - ry)
                if dist >= lookahead_dist:
                    break
                lookahead_idx += 1

            target_x = self.path_x[lookahead_idx]
            target_y = self.path_y[lookahead_idx]
            target_v = self.path_v[self.current_path_index] 

            alpha = math.atan2(target_y - ry, target_x - rx) - ryaw
            alpha = math.atan2(math.sin(alpha), math.cos(alpha)) 

            curvature = (2.0 * math.sin(alpha)) / lookahead_dist
            angular_velocity = target_v * curvature

            twist = Twist()
            twist.linear.x = float(target_v)
            twist.angular.z = float(angular_velocity)
            self.cmd_vel_pub.publish(twist)
            return

        # ----------------------------------------------------
        # PHASE 3: THE MANUAL REVERSE OVERRIDE
        # ----------------------------------------------------
        elif self.mission_state == 'REVERSING_OUT_OF_PIT':
            q = msg.pose.pose.orientation
            siny_cosp = 2 * (q.w * q.z + q.x * q.y)
            cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
            current_yaw = math.atan2(siny_cosp, cosy_cosp)
            
            if current_yaw < -0.05:
                twist = Twist()
                twist.linear.x = -0.5  
                twist.angular.z = 0.1  
                self.cmd_vel_pub.publish(twist)
            else:
                self.cmd_vel_pub.publish(Twist())
                self.set_and_publish_state('NAVIGATING')
                self.get_logger().info(f'Peruutettu lumikasalta. (Yaw: {current_yaw:.2f}). Ajetaan talliin')
                self.start_exit_maneuver_3c()
            return


    # ============================================================
    # PHASE 1A: ESCAPING THE TUNNEL
    # ============================================================
    def start_phase_1_transit(self):
        self.get_logger().info('Vaihe 1A: Ajetaan tallista jäälle')
        self._nav_to_pose_client.wait_for_server()
        
        self.set_and_publish_state('TRANSITING_ESCAPE')
        
        target_x = -27.5
        target_y = -10.25 

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = float(target_x)
        goal_msg.pose.pose.position.y = float(target_y)
        goal_msg.pose.pose.orientation = get_quaternion_from_yaw(0.0)

        send_goal_future = self._nav_to_pose_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self._transit_1a_goal_response_callback)

    def _transit_1a_goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Phase 1A Goal Rejected!')
            self.set_and_publish_state('IDLE')
            return
        get_result_future = goal_handle.get_result_async()
        get_result_future.add_done_callback(self._transit_1a_result_callback)

    def _transit_1a_result_callback(self, future):
        result = future.result()
        if result.status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('Jääkone jäällä. Siirrytään 1B vaiheeseen')
            self.start_phase_1b_staging()

    # ============================================================
    # PHASE 1B: NAVIGATE TO STAGING (Center Ice)
    # ============================================================
    def start_phase_1b_staging(self):
        self.set_and_publish_state('TRANSITING_STAGING')
        
        target_x = -13.0
        target_y = 0.0 - LANE_SPACING 

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = float(target_x)
        goal_msg.pose.pose.position.y = float(target_y)
        goal_msg.pose.pose.orientation = get_quaternion_from_yaw(0.0)

        send_goal_future = self._nav_to_pose_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self._transit_1b_goal_response_callback)

    def _transit_1b_goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.set_and_publish_state('IDLE')
            return
        get_result_future = goal_handle.get_result_async()
        get_result_future.add_done_callback(self._transit_1b_result_callback)

    def _transit_1b_result_callback(self, future):
        result = future.result()
        if result.status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('Vaihe 1B valmis. Valmistaudutaan jäänajoon...')
            self.start_phase_2_resurfacing()

    # ============================================================
    # PHASE 2: INITIATING RIGID RESURFACING
    # ============================================================
    def start_phase_2_resurfacing(self):
        self.get_logger().info('Vaihe 2: Jäänajo aloitettu...')
        px, py, pv = generate_zamboni_path()

        # self.set_conditioner_position(0.2)
        self.path_x = np.array(px)
        self.path_y = np.array(py)
        self.path_v = np.array(pv)
        self.current_path_index = 0
        
        # This unlocks the odom_callback's Phase 2 block!
        self.set_and_publish_state('RESURFACING')

    # ============================================================
    # PHASE 3A: NAVIGATE OUT OF RINK
    # ============================================================
    def start_exit_maneuver_3a(self):
        self.get_logger().info('Vaihe 3A. Ajetaan ulos jäältä')

        target_x = -34.5
        target_y = -10.25
        target_yaw = math.pi  

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = float(target_x)
        goal_msg.pose.pose.position.y = float(target_y)
        goal_msg.pose.pose.orientation = get_quaternion_from_yaw(target_yaw)

        send_goal_future = self._nav_to_pose_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self._exit_goal_3a_response_callback)

    def _exit_goal_3a_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Exit goal rejected by Nav2!')
            return
        get_result_future = goal_handle.get_result_async()
        get_result_future.add_done_callback(self._exit_result_3a_callback)

    def _exit_result_3a_callback(self, future):
        result = future.result()
        if result.status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('Vaihe 3A valmis. Jääkone on tallissa')
            self.start_exit_maneuver_3b()

    # ============================================================
    # PHASE 3B: NAVIGATE TO SNOW UNLOAD STATION
    # ============================================================
    def start_exit_maneuver_3b(self):
        self.get_logger().info('Vaihe 3B: Ajetaan lumentyhjäyspaikalle')

        target_x = -41.50
        target_y = -19.5 # Changed from -20.0 to -19.0 so it doesn't drive too far.
        target_yaw = -math.pi / 2

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = float(target_x)
        goal_msg.pose.pose.position.y = float(target_y)
        goal_msg.pose.pose.orientation = get_quaternion_from_yaw(target_yaw)
        
        send_goal_future = self._nav_to_pose_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self._exit_goal_3b_response_callback)

    def _exit_goal_3b_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Exit goal rejected by Nav2!')
            return
        get_result_future = goal_handle.get_result_async()
        get_result_future.add_done_callback(self._exit_result_3b_callback)

    def _exit_result_3b_callback(self, future):
        result = future.result()
        if result.status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('Tyhjennetään lumisäilötä')
            self.cmd_vel_pub.publish(Twist())
            self.delay_timer = self.create_timer(5.0, self._trigger_manual_reverse)

    def _trigger_manual_reverse(self):
        self.delay_timer.cancel()
        self.get_logger().info('Lumisäiliö tyhjennetty. Peruutetaan ja käännetään jääkone')
        # This unlocks the odom_callback's Phase 3 block!
        self.set_and_publish_state('REVERSING_OUT_OF_PIT')

    # ============================================================
    # PHASE 3C: FINAL PARK IN GARAGE
    # ============================================================
    def start_exit_maneuver_3c(self):
        self.get_logger().info('Vaihe 3C. Ajetaan takaisin talliin')

        target_x = -34.5
        target_y = -10.25
        target_yaw = 0.0

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = float(target_x)
        goal_msg.pose.pose.position.y = float(target_y)
        goal_msg.pose.pose.orientation = get_quaternion_from_yaw(target_yaw)

        send_goal_future = self._nav_to_pose_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self._exit_goal_3c_response_callback)

    def _exit_goal_3c_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Parking goal rejected by Nav2!')
            return
        get_result_future = goal_handle.get_result_async()
        get_result_future.add_done_callback(self._exit_result_3c_callback)

    def _exit_result_3c_callback(self, future):
        result = future.result()
        if result.status == GoalStatus.STATUS_SUCCEEDED:
            self.set_and_publish_state('IDLE')
            self.is_running = False
            self.get_logger().info('Valmista tuli. Jää on ajettu ja jääkone tallissa')

def main(args=None):
    rclpy.init(args=args)
    client = ZamboniMasterNode()
    
    # Kick off Phase 1 immediately
    #def run_once():
    #    client.init_timer.cancel()
    #    client.start_phase_1_transit()

    #client.init_timer = client.create_timer(5.0, run_once)
    
    rclpy.spin(client)
    client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()