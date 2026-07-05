#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TwistStamped

class DriveBridge(Node):
    def __init__(self):
        super().__init__('drive_bridge')
        
        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel', 
            self.listener_callback,
            10)
        
        self.publisher = self.create_publisher(
            TwistStamped,
            '/ackermann_steering_controller/reference',
            10)
            
        print("BRIDGE ACTIVE: Forwarding /cmd_vel -> /ackermann_steering_controller/reference", flush=True)

    def listener_callback(self, msg):
        stamped = TwistStamped()
        stamped.header.stamp = self.get_clock().now().to_msg()
        stamped.header.frame_id = 'base_link' 
        stamped.twist = msg
        self.publisher.publish(stamped)

def main(args=None):
    rclpy.init(args=args)
    bridge = DriveBridge()
    try:
        rclpy.spin(bridge)
    except KeyboardInterrupt:
        pass
    bridge.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()