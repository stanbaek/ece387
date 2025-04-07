#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import cv2
import argparse
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import dlib
from pathlib import Path

class StopDetector(Node):

    def __init__(self, detector_loc):
        super().__init__('stop_detector')
        
        self.ctrl_c = False
        self.bridge = CvBridge()
        
        # Load the detector
        self.detector = dlib.simple_object_detector(detector_loc)
        self.get_logger().info(f"Loaded detector from: {detector_loc}")
        
        # Create subscriber to image topic
        self.subscription = self.create_subscription(
            Image,
            '/image_raw',
            self.camera_callback,
            10)
        self.subscription  # Prevent unused variable warning
        
        # Register shutdown hook
        self.declare_parameter('detector', '')
        self.get_logger().info("Stop detector node ready")

    def camera_callback(self, msg):
        if self.ctrl_c:
            return
            
        try:
            # Convert ROS image to OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            
            # Detect stop signs
            detections = self.detector(cv_image)
            
            # Draw bounding boxes
            for detection in detections:
                l, t, r, b = detection.left(), detection.top(), detection.right(), detection.bottom()
                cv2.rectangle(cv_image, (l, t), (r, b), (0, 0, 255), 2)
                cv2.putText(cv_image, "STOP", (l, t-10), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 0, 255), 2)
            
            # Display image
            cv2.imshow("Stop Sign Detection", cv_image)
            cv2.waitKey(1)
            
        except Exception as e:
            self.get_logger().error(f"Error processing image: {str(e)}")

    def shutdownhook(self):
        self.get_logger().info("Shutting down")
        self.ctrl_c = True
        cv2.destroyAllWindows()

def main(args=None):
    rclpy.init(args=args)
    
    parser = argparse.ArgumentParser()
    parser.add_argument("-d", "--detector", required=True,
                      help="HOG Detector file")
    args, _ = parser.parse_known_args()
        
    stop_detector = StopDetector(args.detector)
    
    try:
        rclpy.spin(stop_detector)
    except KeyboardInterrupt:
        pass
    finally:
        stop_detector.shutdownhook()
        stop_detector.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()