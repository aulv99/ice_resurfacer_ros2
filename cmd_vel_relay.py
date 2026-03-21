import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TwistStamped

class SpinalCord(Node):
    def __init__(self):
        super().__init__('spinal_cord')
        # Listen to Nav2
        self.sub = self.create_subscription(Twist, '/cmd_vel', self.callback, 10)
        # Talk to the Wheels
        self.pub = self.create_publisher(TwistStamped, '/ackermann_steering_controller/reference', 10)
        self.get_logger().info("Spinal cord online: Translating Nav2 commands to Ackermann wheels!")

    def callback(self, msg):
        out = TwistStamped()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = 'base_link'
        out.twist = msg
        self.pub.publish(out)

def main():
    rclpy.init()
    rclpy.spin(SpinalCord())

if __name__ == '__main__':
    main()