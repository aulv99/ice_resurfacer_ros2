import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import tf2_ros
import numpy as np
import math

class ObstacleDetection(Node):
    def __init__(self):
        super().__init__('obstacle_detection')

        # Zamboni dimensions
        self.half_width = 1.06 # around half of 2.13
        self.front_bumper = 2.5

        # Safety areas
        self.stop_distance = 1.5 # hard stop when obstacle 1.5 meters ahead
        self.caution_distance = 3.0 # slow down when obstacle 3.0 meters ahead

        # Rink Geometry
        self.board_margin = 0.1
        self.rink_half_length = 30.0 - self.board_margin
        self.rink_half_width = 15.0 - self.board_margin
        self.corner_radius = 8.5
        

        # TF2 Setup
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Subscribers
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)

        # Tracking current warning state
        self.current_alert_level = 'CLEAR'

        self.get_logger().info("Obstacle Monitor Active.")

    # Helper function to extract yaw from TF2 quaternion
    def get_yaw_from_quaternion(self, q):
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    # Helper function to mathematically rotate and translate NumPY arrays
    def apply_transform(self, x_array, y_array, transform):
        yaw = self.get_yaw_from_quaternion(transform.transform.rotation)
        tx = transform.transform.translation.x
        ty = transform.transform.translation.y
        
        x_trans = x_array * math.cos(yaw) - y_array * math.sin(yaw) + tx
        y_trans = x_array * math.sin(yaw) + y_array * math.cos(yaw) + ty
        return x_trans, y_trans

    def scan_callback(self, msg):
        # get the offset between the physical lidar and the robots pivot point
        try:
            trans_base = self.tf_buffer.lookup_transform(
                'base_link', msg.header.frame_id, rclpy.time.Time()
            )
            trans_map = self.tf_buffer.lookup_transform(
                'map', msg.header.frame_id, rclpy.time.Time()
            )
        except tf2_ros.TransformException as ex:
            return

        # Vectorised math. Convert raw lidar ranges to angles
        ranges = np.array(msg.ranges)
        angles = msg.angle_min + np.arange(len(ranges)) * msg.angle_increment

        # filter out infinite/invalid ranges that lasers often output for empty space
        valid_mask = np.isfinite(ranges) & (ranges >= msg.range_min) & (ranges <= msg.range_max)
        ranges = ranges[valid_mask]
        angles = angles[valid_mask]

        if len(ranges) == 0:
            return

        # Polar to Cartesian
        x_laser = ranges * np.cos(angles)
        y_laser = ranges * np.sin(angles)

        x_base, y_base = self.apply_transform(x_laser, y_laser, trans_base)
        x_map, y_map = self.apply_transform(x_laser, y_laser, trans_map)

        # Map frame filtering: Calculate if points are on the ice area
        abs_x = np.abs(x_map)
        abs_y = np.abs(y_map)

        straight_mask = (abs_x <= (self.rink_half_length - self.corner_radius)) | (abs_y <= (self.rink_half_width - self.corner_radius))
                        
        corner_dist = np.hypot(abs_x - (self.rink_half_length - self.corner_radius), abs_y - (self.rink_half_width - self.corner_radius))
        corner_mask = corner_dist <= self.corner_radius

        on_ice_mask = (abs_x <= self.rink_half_length) & (abs_y <= self.rink_half_width) & (straight_mask | corner_mask)

        # Define the forward swept path mask
        in_path_mask = np.abs(y_base) < self.half_width
        forward_mask = x_base > self.front_bumper

        stop_x = self.front_bumper + self.stop_distance
        caution_x = self.front_bumper + self.caution_distance

        # Check stop zone
        stop_mask = in_path_mask & forward_mask & (x_base <= stop_x) & on_ice_mask
        if np.any(stop_mask):
            if self.current_alert_level != 'STOP':
                self.get_logger().error("Este pysähtymisalueella.")
                self.current_alert_level = 'STOP'
            return

        # Check caution zone
        caution_mask = in_path_mask & forward_mask & (x_base > stop_x) & (x_base <= caution_x) & on_ice_mask
        if np.any(caution_mask):
            if self.current_alert_level != 'CAUTION':
                self.get_logger().warn("Este varoalueella.")
                self.current_alert_level = 'CAUTION'

            return

        # Path clear
        if self.current_alert_level != 'CLEAR':
            self.get_logger().info("Reitti vapaa.")
            self.current_alert_level = 'CLEAR'

def main(args=None):
        rclpy.init(args= args)
        node = ObstacleDetection()
        rclpy.spin(node)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()


