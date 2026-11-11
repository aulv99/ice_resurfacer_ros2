#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from ultralytics import YOLO

class YoloVisionNode(Node):
    def __init__(self):
        super().__init__('vision_node')
        
        self.get_logger().info("Ladataan YOLOv8-mallia... (Tämä saattaa kestää hetken ensimmäisellä kerralla)")
        
        # Load the YOLOv8 nano model (fastest, best for CPU/Edge processing)
        self.model = YOLO('yolov8n.pt') 
        self.bridge = CvBridge()
        
        # Subscribe to the raw Gazebo camera stream
        self.image_sub = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )
        
        # Publish the annotated image (with bounding boxes)
        self.annotated_pub = self.create_publisher(Image, '/camera/image_annotated', 10)
        
        self.get_logger().info("YOLO-konenäkö aktivoitu! Odotetaan kuvaa...")

    def image_callback(self, msg):
        try:
            # 1. Convert ROS Image message to OpenCV format
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
            # 2. Run YOLOv8 Inference
            # verbose=False keeps your terminal clean from spamming every frame
            results = self.model(cv_image, verbose=False)
            
            # 3. Extract the annotated frame (YOLO automatically draws boxes and labels)
            annotated_frame = results[0].plot()
            
            # 4. Foundation for Autonomy (Checking what it saw)
            # You can use this block later to send specific commands to autonomy_seq.py
            for box in results[0].boxes:
                class_id = int(box.cls[0])
                class_name = self.model.names[class_id]
                
                # If YOLO sees a person, you could theoretically publish a "HALT" 
                # command to a new topic here!
                if class_name == 'person':
                    pass # self.get_logger().warn("Ihminen havaittu!")
            
            # 5. Convert back to ROS Image and publish to RViz/Web UI
            annotated_msg = self.bridge.cv2_to_imgmsg(annotated_frame, encoding='bgr8')
            annotated_msg.header = msg.header # Keep original timestamp
            self.annotated_pub.publish(annotated_msg)
            
        except Exception as e:
            self.get_logger().error(f"Virhe kuvan käsittelyssä: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = YoloVisionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()