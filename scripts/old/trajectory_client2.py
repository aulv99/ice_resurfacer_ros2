import math
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped, Quaternion
from nav_msgs.msg import Path, Odometry
from nav2_msgs.action import FollowPath
from nav2_msgs.msg import SpeedLimit
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy, qos_profile_sensor_data

# ============================================================
# RINK & ZAMBONI PARAMETERS
# ============================================================
RINK_LENGTH = 60.0
RINK_WIDTH = 30.0
CORNER_RADIUS = 8.5
CONDITIONER_WIDTH = 2.13
LANE_SPACING = 2.0          
SAFETY_MARGIN = 0.10
POINT_SPACING = 0.10

# --- THE SPEED PROFILES ---
WALL_STRAIGHT_SPEED = 1.0
WALL_CORNER_SPEED = 0.5
SWEEP_STRAIGHT_SPEED = 1.5
SWEEP_CORNER_SPEED = 1.0

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
    # Calculates exactly how far out the robot should be based on how close it is to the corner
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
    # Append the requested speed limit for every single point in this segment
    path_v.extend([target_speed] * len(seg_x))

def generate_wall_lane_change(start_x, start_y, end_x, n_points=40):
    # Smooth S-Curve that bridges the exact gap between the dynamic Lap 0 and the rigid Lap 1
    local_x = np.linspace(0.0, end_x - start_x, n_points)
    progress = np.linspace(0, math.pi, n_points)
    local_y = (6.0 / 2.0) * (1 - np.cos(progress))
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
    # Give this transition corner the slow wall-corner speed
    append_segment(x, y, v, sx, sy, WALL_CORNER_SPEED)
    
    return x, y, v, y_wall_end

# ============================================================
# SWEEP GENERATOR
# ============================================================
def generate_classic_zamboni_sweeps(x_limit, y_top_start, y_bottom_start, lane_spacing, num_passes):
    x = []
    y = []
    v = []
    current_y_top = y_top_start
    current_y_bottom = y_bottom_start

    for i in range(num_passes):
        # 1. Drive East (Top Lane)
        sx, sy = sample_line((-x_limit, current_y_top), (x_limit, current_y_top))
        append_segment(x, y, v, sx, sy, SWEEP_STRAIGHT_SPEED)

        # 2. Right Semicircle at East end
        radius_east = (current_y_top - current_y_bottom) / 2.0
        center_y_east = (current_y_top + current_y_bottom) / 2.0
        sx, sy = sample_arc((x_limit, center_y_east), radius_east, math.pi/2, -math.pi/2)
        append_segment(x, y, v, sx, sy, SWEEP_CORNER_SPEED)

        # 3. Drive West (Bottom Lane)
        sx, sy = sample_line((x_limit, current_y_bottom), (-x_limit, current_y_bottom))
        append_segment(x, y, v, sx, sy, SWEEP_STRAIGHT_SPEED)

        if i == num_passes - 1:
            break

        # 4. Right Semicircle at West end
        next_y_top = current_y_top + lane_spacing
        radius_west = (next_y_top - current_y_bottom) / 2.0
        center_y_west = (next_y_top + current_y_bottom) / 2.0
        sx, sy = sample_arc((-x_limit, center_y_west), radius_west, -math.pi/2, -3*math.pi/2)
        append_segment(x, y, v, sx, sy, SWEEP_CORNER_SPEED)

        current_y_top = next_y_top
        current_y_bottom += lane_spacing

    return x, y, v

# ============================================================
# OFFSET RINK LAP 
# ============================================================
def generate_wall_lap(offset_straight, offset_corner, start_y=None, end_y=None):
    x, y, v = [], [], []

    r = CORNER_RADIUS - offset_corner

    cx_tl, cy_tl = -30 + CORNER_RADIUS, 15 - CORNER_RADIUS
    cx_tr, cy_tr = 30 - CORNER_RADIUS, 15 - CORNER_RADIUS
    cx_br, cy_br = 30 - CORNER_RADIUS, -15 + CORNER_RADIUS
    cx_bl, cy_bl = -30 + CORNER_RADIUS, -15 + CORNER_RADIUS

    if start_y is None: start_y = cy_bl
    if end_y is None: end_y = cy_bl

    # 1. ENTRY STRAIGHT (Drifts from corner margin to straight margin)
    sx, sy = sample_dynamic_line_y_major(start_y, cy_tl, cy_bl, cy_tl, -30.0, offset_straight, offset_corner, +1)
    append_segment(x, y, v, sx, sy, WALL_STRAIGHT_SPEED)

    # 2. Top Left Corner (Rigid safe margin)
    sx, sy = sample_arc((cx_tl, cy_tl), r, math.pi, math.pi/2)
    append_segment(x, y, v, sx, sy, WALL_CORNER_SPEED)

    # 3. Top Straight 
    sx, sy = sample_dynamic_line_x_major(cx_tl, cx_tr, cx_tl, cx_tr, 15.0, offset_straight, offset_corner, -1)
    append_segment(x, y, v, sx, sy, WALL_STRAIGHT_SPEED)

    # 4. Top Right Corner
    sx, sy = sample_arc((cx_tr, cy_tr), r, math.pi/2, 0)
    append_segment(x, y, v, sx, sy, WALL_CORNER_SPEED)

    # 5. Right Straight
    sx, sy = sample_dynamic_line_y_major(cy_tr, cy_br, cy_tr, cy_br, 30.0, offset_straight, offset_corner, -1)
    append_segment(x, y, v, sx, sy, WALL_STRAIGHT_SPEED)

    # 6. Bottom Right Corner
    sx, sy = sample_arc((cx_br, cy_br), r, 0, -math.pi/2)
    append_segment(x, y, v, sx, sy, WALL_CORNER_SPEED)

    # 7. Bottom Straight
    sx, sy = sample_dynamic_line_x_major(cx_br, cx_bl, cx_br, cx_bl, -15.0, offset_straight, offset_corner, +1)
    append_segment(x, y, v, sx, sy, WALL_STRAIGHT_SPEED)

    # 8. Bottom Left Corner
    sx, sy = sample_arc((cx_bl, cy_bl), r, -math.pi/2, -math.pi)
    append_segment(x, y, v, sx, sy, WALL_CORNER_SPEED)

    # 9. EXIT STRAIGHT
    sx, sy = sample_dynamic_line_y_major(cy_bl, end_y, cy_bl, cy_tl, -30.0, offset_straight, offset_corner, +1)
    append_segment(x, y, v, sx, sy, WALL_STRAIGHT_SPEED)

    return x, y, v

# ============================================================
# MASTER GENERATOR
# ============================================================
def generate_zamboni_path():
    px, py, pv = [], [], []

    # THE DYNAMIC MARGINS
    margin_straight = 0.05
    margin_corner = 0.12
    first_offset_straight = CONDITIONER_WIDTH/2 + margin_straight
    first_offset_corner = CONDITIONER_WIDTH/2 + margin_corner

    # ==========================================
    # 1. LAP 0 (The Outer Lap)
    # ==========================================
    lx0, ly0, lv0 = generate_wall_lap(
        offset_straight=first_offset_straight, 
        offset_corner=first_offset_corner, 
        start_y=-6.5, 
        end_y=-5.0
    )
    px.extend(lx0); py.extend(ly0); pv.extend(lv0)

    # ==========================================
    # 2. TRANSITION 1 (Lane Change: Lap 0 to Lap 1)
    # ==========================================
    # Find exact X coordinate where Lap 0 finishes drifting out
    lap0_end_x = calculate_dynamic_coord(-5.0, -6.5, 6.5, -30.0, first_offset_straight, first_offset_corner, 1)
    
    # Lap 1 is safe, so it just uses a static margin offset
    offset_1 = first_offset_straight + LANE_SPACING 
    lap1_start_x = -30.0 + offset_1 

    tx1, ty1 = generate_wall_lane_change(
        start_x=lap0_end_x,
        start_y=-5.0,
        end_x=lap1_start_x
    )
    append_segment(px, py, pv, tx1, ty1, WALL_CORNER_SPEED)

    # ==========================================
    # 3. PREPARE FOR LAP 1 & SWEEPS
    # ==========================================
    x_wall_lap_1 = lap1_start_x  
    y_sweep_top_start = 1.0          
    x_sweep_start = -20.0            
    sweep_x_limit = 20.0             
    
    tx2, ty2, tv2, lap1_end_y = generate_wall_to_sweep_transition(
        x_wall=x_wall_lap_1, 
        y_sweep_top=y_sweep_top_start, 
        x_sweep_start=x_sweep_start
    )

    # ==========================================
    # 4. LAP 1 (The Inner Lap)
    # ==========================================
    lx1, ly1, lv1 = generate_wall_lap(
        offset_straight=offset_1, 
        offset_corner=offset_1, 
        start_y=1.0, 
        end_y=lap1_end_y
    )
    px.extend(lx1); py.extend(ly1); pv.extend(lv1)

    # ==========================================
    # 5. TRANSITION 2 (90-Degree Turn into Sweeps)
    # ==========================================
    px.extend(tx2); py.extend(ty2); pv.extend(tv2)

    # ==========================================
    # 6. THE INNER SWEEPS
    # ==========================================
    sweep_x, sweep_y, sweep_v = generate_classic_zamboni_sweeps(
        x_limit=sweep_x_limit,       
        y_top_start=y_sweep_top_start,        
        y_bottom_start=-10.4,       
        lane_spacing=LANE_SPACING,       
        num_passes=6            
    )
    
    px.extend(sweep_x); py.extend(sweep_y); pv.extend(sweep_v)

    return px, py, pv

# ============================================================
# ROS 2 CLIENT NODE
# ============================================================
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
        self._action_client = ActionClient(self, FollowPath, 'follow_path')
        
        
        speed_limit_qos = QoSProfile(
            depth=1,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            reliability=QoSReliabilityPolicy.RELIABLE
        )
        self.speed_limit_pub = self.create_publisher(SpeedLimit, '/speed_limit', speed_limit_qos)
        
        
        self.odom_sub = self.create_subscription(
            Odometry, 
            '/odometry/filtered', 
            self.odom_callback, 
            qos_profile_sensor_data
        )
        
        self.path_x = None
        self.path_y = None
        self.path_v = None
        self.current_speed_limit = -1.0
        self.path_active = False

    def odom_callback(self, msg):
        if not self.path_active: return

        # Get current position
        rx = msg.pose.pose.position.x
        ry = msg.pose.pose.position.y

        # Find the closest point on our path
        dx = self.path_x - rx
        dy = self.path_y - ry
        distances = dx**2 + dy**2
        closest_idx = np.argmin(distances)

        # Look ahead 20 points (2.0 meters) to brake BEFORE entering the corner!
        lookahead_idx = min(closest_idx + 20, len(self.path_v) - 1)
        
        # Find the minimum speed limit in that upcoming window
        target_v = np.min(self.path_v[closest_idx : lookahead_idx + 1])

        # Publish only if the speed limit needs to change
        if target_v != self.current_speed_limit:
            limit_msg = SpeedLimit()
            limit_msg.header.stamp = self.get_clock().now().to_msg()
            limit_msg.header.frame_id = 'map'
            limit_msg.speed_limit = float(target_v)
            limit_msg.percentage = False
            
            self.speed_limit_pub.publish(limit_msg)
            self.current_speed_limit = target_v
            self.get_logger().info(f'Dynamic Speed Zone Updated -> {target_v} m/s')

    def send_path(self):
        self.get_logger().info('Generating velocity-profiled trajectory...')
        px, py, pv = generate_zamboni_path()
        
        # Store as numpy arrays for lightning-fast odometry lookups
        self.path_x = np.array(px)
        self.path_y = np.array(py)
        self.path_v = np.array(pv)

        path_msg = Path()
        path_msg.header.frame_id = 'map'
        path_msg.header.stamp = self.get_clock().now().to_msg()
        
        for i in range(len(px) - 1):
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.header.stamp = path_msg.header.stamp
            
            pose.pose.position.x = float(px[i])
            pose.pose.position.y = float(py[i])
            pose.pose.position.z = 0.0

            dy = py[i+1] - py[i]
            dx = px[i+1] - px[i]
            yaw = math.atan2(dy, dx)
            pose.pose.orientation = get_quaternion_from_yaw(yaw)
            path_msg.poses.append(pose)

        # Final pose logic
        final_pose = PoseStamped()
        final_pose.header = path_msg.poses[-1].header
        final_pose.pose.position.x = float(px[-1])
        final_pose.pose.position.y = float(py[-1])
        final_pose.pose.orientation = path_msg.poses[-1].pose.orientation
        path_msg.poses.append(final_pose)

        self.get_logger().info('Waiting for FollowPath action server...')
        self._action_client.wait_for_server()

        goal_msg = FollowPath.Goal()
        goal_msg.path = path_msg

        self.get_logger().info('Sending complete Zamboni trajectory to Nav2!')
        self._action_client.send_goal_async(goal_msg)
        
        # Activate the live radar!
        self.path_active = True

def main(args=None):
    rclpy.init(args=args)
    client = ZamboniPathClient()
    client.send_path()
    rclpy.spin(client)

if __name__ == '__main__':
    main()