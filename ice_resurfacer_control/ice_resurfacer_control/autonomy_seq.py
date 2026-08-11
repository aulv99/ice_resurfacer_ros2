import math
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped, Quaternion, Twist
from nav_msgs.msg import Path, Odometry, OccupancyGrid
from nav2_msgs.action import FollowPath, NavigateToPose
from nav2_msgs.msg import SpeedLimit
from action_msgs.msg import GoalStatus
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy, qos_profile_sensor_data
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from std_msgs.msg import String, Float32
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformListener
from tf2_ros import TransformException

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
WALL_STRAIGHT_SPEED = 1.5
WALL_CORNER_SPEED = 1.0
SWEEP_STRAIGHT_SPEED = 2.0
SWEEP_CORNER_SPEED = 1.5
STAGING_SPEED = 0.5

# ============================================================
# GEOMETRY HELPERS 
# ============================================================

# sample_line draws a simple straight line between two points and chops it up into 10 cm dots
# takes two points and point spacing as parameters and return a list of points
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

# sample_arc draws a curved circular path and chops it into 10 cm dots
def sample_arc(center, radius, start_angle, end_angle, spacing=POINT_SPACING):
    arc_len = radius * abs(end_angle - start_angle)
    if arc_len == 0:
        return [center[0] + radius * math.cos(start_angle)], [center[1] + radius * math.sin(start_angle)]
    n = max(5, int(arc_len / spacing))
    angles = np.linspace(start_angle, end_angle, n)
    xs = center[0] + radius * np.cos(angles)
    ys = center[1] + radius * np.sin(angles)
    return xs.tolist(), ys.tolist()

# calculate_dynamic_coord is a "taper" calculator. It calculates exactly how far the robot 
# should drift outward as it drives down a straightaway toward a corner
def calculate_dynamic_coord(val, val_start, val_end, base_coord, offset_s, offset_c, sign):
    mid_val = (val_start + val_end) / 2.0
    max_dist = abs(val_end - mid_val)
    if max_dist == 0: return base_coord + sign * offset_s
    
    ratio = abs(val - mid_val) / max_dist
    current_offset = offset_s + (offset_c - offset_s) * ratio
    return base_coord + sign * current_offset

# sample_dynamic_line_y_major draws a vertical straigh line (like the left/right rink boards)
# where the X coordinate slowly shifts using the taper calculator
def sample_dynamic_line_y_major(y1, y2, y_full_start, y_full_end, base_x, offset_s, offset_c, sign, spacing=POINT_SPACING):
    length = abs(y2 - y1)
    if length == 0:
        x = calculate_dynamic_coord(y1, y_full_start, y_full_end, base_x, offset_s, offset_c, sign)
        return [x], [y1]
    n = max(2, int(length / spacing))
    ys = np.linspace(y1, y2, n)
    xs = [calculate_dynamic_coord(y, y_full_start, y_full_end, base_x, offset_s, offset_c, sign) for y in ys]
    return xs, ys.tolist()

# sample_dynamic_line_x_major draws a vertical straigh line (like the left/right rink boards)
# where the Y coordinate slowly shifts using the taper calculator
def sample_dynamic_line_x_major(x1, x2, x_full_start, x_full_end, base_y, offset_s, offset_c, sign, spacing=POINT_SPACING):
    length = abs(x2 - x1)
    if length == 0:
        y = calculate_dynamic_coord(x1, x_full_start, x_full_end, base_y, offset_s, offset_c, sign)
        return [x1], [y]
    n = max(2, int(length / spacing))
    xs = np.linspace(x1, x2, n)
    ys = [calculate_dynamic_coord(x, x_full_start, x_full_end, base_y, offset_s, offset_c, sign) for x in xs]
    return xs.tolist(), ys

# append_segment attaches a new segment of dots to the master array while assigning a specific
# speed to the segment. Deletes first dot to prevent overlapping duplicates
def append_segment(path_x, path_y, path_v, seg_x, seg_y, target_speed):
    if len(path_x) > 0 and len(seg_x) > 0:
        seg_x = seg_x[1:]
        seg_y = seg_y[1:]
    path_x.extend(seg_x)
    path_y.extend(seg_y)
    path_v.extend([target_speed] * len(seg_x))

# ============================================================
# RESURFACING PATTERNS
# ============================================================

# generate_wall_lane_change_horizontal creates a smooth cosine-based S-curve to shift the
# Zamboni sideways into the next lane
def generate_wall_lane_change_horizontal(start_x, end_x, start_y, end_y, n_points=40):
    local_x = np.linspace(0.0, end_x - start_x, n_points)
    progress = np.linspace(0, math.pi, n_points)
    
    y_shift = end_y - start_y
    local_y = (y_shift / 2.0) * (1 - np.cos(progress))
    
    x = start_x + local_x
    y = start_y + local_y
    return x.tolist(), y.tolist()

# generate_wall_to_sweep_transition Calculates a perfect 90-degree corner to move the robot 
# off the outer wall laps and align it for the inner sweeping pattern.
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

# generate_staging_alignment calculates the very first maneuver: a U-turn and lane shift
#  that aligns the robot from the entry gate onto the bottom straightaway starting dot.  
def generate_staging_alignment(staging_x, staging_y, target_x, target_y):
    x, y, v = [], [], []
    u_turn_start_x = -10.0
    if staging_x < u_turn_start_x:
        sx, sy = sample_line((staging_x, staging_y), (u_turn_start_x, staging_y))
        append_segment(x, y, v, sx, sy, STAGING_SPEED)
        
    u_turn_end_y = -13.0
    radius = abs(staging_y - u_turn_end_y) / 2.0
    center_x = u_turn_start_x
    center_y = (staging_y + u_turn_end_y) / 2.0
    
    sx, sy = sample_arc((center_x, center_y), radius, math.pi/2, -math.pi/2)
    append_segment(x, y, v, sx, sy, STAGING_SPEED)
    
    tx, ty = generate_wall_lane_change_horizontal(
        start_x=u_turn_start_x, end_x=target_x, start_y=u_turn_end_y, end_y=target_y, n_points=60
    )
    append_segment(x, y, v, tx, ty, STAGING_SPEED)
    
    return x, y, v

# generate_classic_zamboni_sweeps ontains the loop that draws the repeating horizontal
# passes with 180-degree right-hand turns to clean the center of the ice.
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

# generate_full_wall_lap_bottom drives exactly one complete lap around the outer boards. 
# It pieces together four straightaways and four corners, starting and ending perfectly on the bottom straight.
def generate_full_wall_lap_bottom(offset_straight, offset_corner, start_x, end_x):
    x, y, v = [], [], []
    r = CORNER_RADIUS - offset_corner

    cx_tl, cy_tl = -30 + CORNER_RADIUS, 15 - CORNER_RADIUS
    cx_tr, cy_tr = 30 - CORNER_RADIUS, 15 - CORNER_RADIUS
    cx_br, cy_br = 30 - CORNER_RADIUS, -15 + CORNER_RADIUS
    cx_bl, cy_bl = -30 + CORNER_RADIUS, -15 + CORNER_RADIUS

    sx, sy = sample_dynamic_line_x_major(start_x, cx_bl, cx_br, cx_bl, -15.0, offset_straight, offset_corner, +1)
    append_segment(x, y, v, sx, sy, WALL_CORNER_SPEED)
    sx, sy = sample_arc((cx_bl, cy_bl), r, -math.pi/2, -math.pi)
    append_segment(x, y, v, sx, sy, STAGING_SPEED)
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

# generate_zamboni_path is the master builder of the resurfacing pattern.
# It calls all the pattern builders in the correct order to generate the entire mission array
def generate_zamboni_path():
    px, py, pv = [], [], []

    margin_straight = 0.10  # 0.03
    margin_corner = 0.10  # 0.06
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

# get_quaternion_from_yaw converts a flat, 2D heading angle into a 3D ROS 2 quaternion (required for Pose messages)
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
        super().__init__('Zamboni_AI')
        
        # --- Action Clients & Publishers ---
        self._nav_to_pose_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self._nav_to_pose_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        # self._conditioner_client = ActionClient(self, FollowJointTrajectory, '/conditioner_controller/follow_joint_trajectory')
        self.state_pub = self.create_publisher(String, '/mission_state', 10)
        self.rviz_path_pub = self.create_publisher(Path, '/zamboni_static_path', 10) # Publisher for static trajectory

        # ROS2 Service Server
        self.start_srv = self.create_service(Trigger, '/start_sequence', self.start_callback)
        self.stop_srv = self.create_service(Trigger, '/stop_sequence', self.stop_callback)

        # --- TF2 Listener ---
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Coverage tracker
        self.coverage_pub = self.create_publisher(OccupancyGrid, '/ice_coverage_map', 1)
        self.coverage_percent_pub = self.create_publisher(Float32, '/ice_coverage_percent', 10)

        # Grid parameters: 
        self.grid_res = 0.1
        self.grid_width = 60.0
        self.grid_height = 30.0
        self.grid_w = int(self.grid_width / self.grid_res)
        self.grid_h = int(self.grid_height / self.grid_res)

        # Grid origin
        self.grid_origin_x = -self.grid_width / 2.0
        self.grid_origin_y = -self.grid_height / 2.0

        # Mask (true if cell is inside the rounded rink)
        self.rink_mask = np.zeros((self.grid_h, self.grid_w), dtype=np.bool_)

        # Filling coverage grid with -1 by default
        self.coverage_grid = np.full((self.grid_h, self.grid_w), -1, dtype=np.int8)
        self.total_ice_cells = 0

        # Pre-calculate the exact rounded rink geometry!
        for y in range(self.grid_h):
            for x in range(self.grid_w):
                wx = self.grid_origin_x + x * self.grid_res
                wy = self.grid_origin_y + y * self.grid_res
                
                abs_x = abs(wx)
                abs_y = abs(wy)
                
                # Check if it's within the 60x30 bounding box
                if abs_x <= 30.0 and abs_y <= 15.0:
                    # Check if it's in the straightaways or the corners
                    if abs_x <= (30.0 - CORNER_RADIUS) or abs_y <= (15.0 - CORNER_RADIUS):
                        self.rink_mask[y, x] = True
                    else:
                        # Pythagorean theorem for the rounded corners
                        dist = math.hypot(abs_x - (30.0 - CORNER_RADIUS), abs_y - (15.0 - CORNER_RADIUS))
                        if dist <= CORNER_RADIUS:
                            self.rink_mask[y, x] = True
                
                # If it's valid ice, mark it as 0 (Uncleaned) and count it
                if self.rink_mask[y, x]:
                    self.coverage_grid[y, x] = 0
                    self.total_ice_cells += 1
                    
        self.coverage_update_counter = 0
        
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

    # start_callback is the service trigger for UI start sequence command
    def start_callback(self, request, response):
        """ Triggered when the UI presses START """
        if self.is_running:
            response.success = False
            response.message = "Jäänajo on jo käynnissä"
            return response
            
        # Add safety checks here
        
        self.is_running = True
        self.get_logger().info("Jäänajo aloitettu ohjauspaneelista")
        
        # Triggering the first phase of sequence
        self.start_phase_1_transit()
        
        response.success = True
        response.message = "Sequence started successfully."
        return response

    # stop_callback is the service trigger for UI stop sequence commmand
    # NOT IN USE
    def stop_callback(self, request, response):
        """ Triggered when the UI presses STOP """
        if not self.is_running:
            response.success = False
            response.message = "Already stopped."
            return response

        self.is_running = False
        self.get_logger().warn("Jäänajo keskeytetty")
        self.set_and_publish_state('HALTED')
        
        # Emergency brake when halting operation
        brake_msg = Twist()
        brake_msg.linear.x = 0.0
        brake_msg.angular.z = 0.0
        self.cmd_vel_pub.publish(brake_msg)

        # Canceling any active Nav2 goals
        if hasattr(self, 'nav_goal_handle') and self.nav_goal_handle is not None:
            self.get_logger().info("Perutaan reitti...")
            self.nav_goal_handle.cancel_goal_async()
        
        response.success = True
        response.message = "Emergency Stop executed."
        return response

    # set_and_publish_state updates the robot's current status (e.g., "IDLE" or "RESURFACING") 
    # and broadcasts it to the ROS network so the UI can display it.
    def set_and_publish_state(self, new_state):
        self.mission_state = new_state
        msg = String()
        msg.data = new_state
        self.state_pub.publish(msg)

        # Dictionary of the sequence phases that 
        state_dict = {
            "PHASE_1A": "1A: Ajetaan jäälle",
            "PHASE_1B": "1B: Jäänajon valmistelu",
            "PHASE_2": "2: Jäänajo",
            "PHASE_3A": "3A: Ajetaan pois jäältä",
            "PHASE_3B": "3B: Ajetaan lumentyhjäyspaikalle",
            "PHASE_3C": "3C: Peruutetaan pois lumentyhjäyspaikalta",
            "PHASE_3D": "3D: Ajetaan takaisin talliin",
            "IDLE": "Valmiudessa",
            "HALTED": "Hätäseis"
        }

        state_description = state_dict.get(new_state, new_state)

        self.get_logger().info(f"Nykyinen tila: {state_description} ")  

    # ============================================================
    # SHARED ODOMETRY CALLBACK (Custom Driver) 
    # ============================================================

    # During Phase 2, odom_callback acts as the Pure Pursuit controller (overriding Nav2), 
    # mathematically steering the robot along the arrays based on its TF2 map position.
    # It also performs the reverse out of snow pit
    def odom_callback(self, msg):
        # ----------------------------------------------------
        # PHASE 2: RESURFACING PURE PURSUIT
        # ----------------------------------------------------

        # When operation is halted, static path planning will not go through
        if self.mission_state == 'HALTED':
            return

        # Starting static resurfacing in phase 2
        if self.mission_state == 'PHASE_2':
            # Ask TF2 for the robot position in the MAP frame 
            # This is to ensure that the static plan will be w.r.t to the map frame
            try: 
                t = self.tf_buffer.lookup_transform(
                    'map',
                    'base_link',
                    rclpy.time.Time()
                ) 
            except TransformException as ex:
                self.get_logger().warn(f'Could not transform base_link to map: {ex}')
                return

            # using map coordinates
            rx = t.transform.translation.x
            ry = t.transform.translation.y
            q = t.transform.rotation

            siny_cosp = 2 * (q.w * q.z + q.x * q.y)
            cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
            ryaw = math.atan2(siny_cosp, cosy_cosp)

            # ----------------
            # Coverage tracker
            # ----------------

            # Conditioner is 1.09m behind the rear axle
            cond_x = rx - 1.09 * math.cos(ryaw)
            cond_y = ry - 1.09 * math.sin(ryaw)

            # Convert world coordinates to array indices
            cx = int((cond_x - self.grid_origin_x) / self.grid_res)
            cy = int((cond_y - self.grid_origin_y) / self.grid_res)

            # Conditioner is 2.13m wide (Radius is ~1.065m)
            radius_cells = int(1.065 / self.grid_res)

            # Paint the cells within the conditioner's radius
            y_min = max(0, cy - radius_cells)
            y_max = min(self.grid_h, cy + radius_cells + 1)
            x_min = max(0, cx - radius_cells)
            x_max = min(self.grid_w, cx + radius_cells + 1)
            
            for y in range(y_min, y_max):
                for x in range(x_min, x_max):
                    if (x - cx)**2 + (y - cy)**2 <= radius_cells**2:
                        if self.rink_mask[y, x]:
                            self.coverage_grid[y, x] = 100

            # Throttle the publisher so we only send the map to RViz at ~5 Hz
            self.coverage_update_counter += 1
            if self.coverage_update_counter % 10 == 0:
                # Calculating completion percentage
                cleaned_cells = np.count_nonzero(self.coverage_grid == 100)
                completion_pct = (cleaned_cells / self.total_ice_cells) * 100.0 if self.total_ice_cells > 0 else 0.0

                # Publish the percentage
                pct_msg = Float32()
                pct_msg.data = float(completion_pct)
                self.coverage_percent_pub.publish(pct_msg)

                # Publish the map
                grid_msg = OccupancyGrid()
                grid_msg.header.stamp = self.get_clock().now().to_msg()
                grid_msg.header.frame_id = 'map'
                grid_msg.info.resolution = self.grid_res
                grid_msg.info.width = self.grid_w
                grid_msg.info.height = self.grid_h
                grid_msg.info.origin.position.x = float(self.grid_origin_x)
                grid_msg.info.origin.position.y = float(self.grid_origin_y)
                
                # Flatten the 2D array back into a 1D list for ROS
                grid_msg.data = self.coverage_grid.flatten().tolist()
                self.coverage_pub.publish(grid_msg)

            # The path array will only show 30 next points to not get confused about 
            # crossing paths
            search_window = 30 # 100 # 30 = 3 meters, 100 = 10 meters
            start_idx = self.current_path_index
            end_idx = min(start_idx + search_window, len(self.path_x))
            
            dx_arr = self.path_x[start_idx:end_idx] - rx
            dy_arr = self.path_y[start_idx:end_idx] - ry
            distances = dx_arr**2 + dy_arr**2
            local_closest = np.argmin(distances)
            
            self.current_path_index = start_idx + local_closest

            # When the Phase 2 is completed, we move to Phase 3A 
            # by calling the start_exit_maneuver_3a function
            if self.current_path_index >= len(self.path_x) - 5:
                self.cmd_vel_pub.publish(Twist())
                # self.set_conditioner_position(-0.2) 
                self.get_logger().info('Jäänajo valmis. Ajetaan pois jäältä')
                self.set_and_publish_state('PHASE_3A')
                self.start_exit_maneuver_3a()
                return

            lookahead_dist = 3.5 # 2.5  # Static lookahead distance

            # Dynamic lookahead distance NOT IN USE
            # actual_v = msg.twist.twist.linear.x # forward velocity from EKF
            # lookahead_dist = max(0.8, min(2.5, actual_v * 1.5))

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

            # Dynamic curvature calculation NOT IN USE
            # Calculating steering with actual velocity
            # calc_v = max(0.1, actual_v)
            # angular_velocity = calc_v * curvature

            twist = Twist()
            twist.linear.x = float(target_v)
            twist.angular.z = float(angular_velocity)
            self.cmd_vel_pub.publish(twist)
            return

        # ----------------------------------------------------
        # PHASE 3C: THE MANUAL REVERSE OVERRIDE
        # ----------------------------------------------------
        elif self.mission_state == 'PHASE_3C':
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
                self.get_logger().info(f'Peruutettu pois lumikasalta. Ajetaan talliin')
                self.set_and_publish_state('PHASE_3D')
                self.start_exit_maneuver_3d()
            return


    # ============================================================
    # PHASE 1A: ESCAPING THE GARAGE
    # ============================================================
    def start_phase_1_transit(self):
        self.get_logger().info('Vaihe 1A: Ajetaan tallista jäälle')
        self._nav_to_pose_client.wait_for_server()
        
        self.set_and_publish_state('PHASE_1A')
        
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
        self.set_and_publish_state('PHASE_1B')
        
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

        # Rviz path visualization
        path_msg = Path()
        path_msg.header.frame_id = 'map'
        path_msg.header.stamp = self.get_clock().now().to_msg()

        # Loop through the arrays and build the PoseStamped messages
        for i in range(len(px) - 1):
            pose = PoseStamped()
            pose.header = path_msg.header
            pose.pose.position.x = float(px[i])
            pose.pose.position.y = float(py[i])
            pose.pose.position.z = 0.0

            # calculating the yaw for Rviz arrows
            dy = py[i+1] - py[i]
            dx = px[i+1] - px[i]
            yaw = math.atan2(dy, dx)
            pose.pose.orientation = get_quaternion_from_yaw(yaw)

            path_msg.poses.append(pose)

        # append the very last point
        if len(px) > 0:
            final_pose = PoseStamped()
            final_pose.header = path_msg.header
            final_pose.pose.position.x = float(px[-1])
            final_pose.pose.position.y = float(py[-1])
            final_pose.pose.orientation = path_msg.poses[-1].pose.orientation if len(path_msg.poses) > 0 else get_quaternion_from_yaw(0.0)
            path_msg.poses.append(final_pose)

        # publish to Rviz
        self.rviz_path_pub.publish(path_msg)

        # self.set_conditioner_position(0.2)
        self.path_x = np.array(px)
        self.path_y = np.array(py)
        self.path_v = np.array(pv)
        self.current_path_index = 0
        
        # This unlocks the odom_callback's Phase 2 block!
        self.set_and_publish_state('PHASE_2')

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
        self.set_and_publish_state('PHASE_3B')

        target_x = -41.50
        target_y = -19.5 # Changed from -20.0 to -19.5 so it doesn't drive too far.
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
            self.get_logger().info('Vaihe 3B valmis. Tyhjennetään lumisäiliötä')
            self.cmd_vel_pub.publish(Twist())
            self.delay_timer = self.create_timer(5.0, self._trigger_manual_reverse)

    def _trigger_manual_reverse(self):
        self.delay_timer.cancel()
        self.get_logger().info('Lumisäiliö tyhjennetty. Peruutetaan ja käännetään jääkone')
        # This unlocks the odom_callback's Phase 3 block!
        self.set_and_publish_state('PHASE_3C')
        self.get_logger().info(f'Vaihe 3C: Peruutetaan ulos lumikasalta')

    # ============================================================
    # PHASE 3C: FINAL PARK IN GARAGE
    # ============================================================
    def start_exit_maneuver_3d(self):
        self.get_logger().info('Vaihe 3D. Ajetaan takaisin talliin')

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
        send_goal_future.add_done_callback(self._exit_goal_3d_response_callback)

    def _exit_goal_3d_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Parking goal rejected by Nav2!')
            return
        get_result_future = goal_handle.get_result_async()
        get_result_future.add_done_callback(self._exit_result_3d_callback)

    def _exit_result_3d_callback(self, future):
        result = future.result()
        if result.status == GoalStatus.STATUS_SUCCEEDED:
            self.set_and_publish_state('IDLE')
            self.is_running = False
            self.get_logger().info('Vaihe 3D Valmis. Jää on ajettu ja jääkone tallissa')

            # Coverage tracker reset
            self.coverage_grid.fill(-1)
            self.coverage_grid[self.rink_mask] = 0
            pct_msg = Float32()
            pct_msg.data = 0.0
            self.coverage_percent_pub.publish(pct_msg)

            # Broadcast the wiped, clean map back to RViz
            grid_msg = OccupancyGrid()
            grid_msg.header.stamp = self.get_clock().now().to_msg()
            grid_msg.header.frame_id = 'map'
            grid_msg.info.resolution = self.grid_res
            grid_msg.info.width = self.grid_w
            grid_msg.info.height = self.grid_h
            grid_msg.info.origin.position.x = float(self.grid_origin_x)
            grid_msg.info.origin.position.y = float(self.grid_origin_y)
            
            grid_msg.data = self.coverage_grid.flatten().tolist()
            self.coverage_pub.publish(grid_msg)

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