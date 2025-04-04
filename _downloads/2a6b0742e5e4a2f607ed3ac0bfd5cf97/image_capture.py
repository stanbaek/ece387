#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import cv2
import argparse
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import threading
import time
import os
from pathlib import Path

class ImageSaver(Node):
    def __init__(self, img_dest):
        super().__init__('image_saver')
        
        # Create output directory if it doesn't exist
        self.img_dest = Path(img_dest)
        self.img_dest.mkdir(parents=True, exist_ok=True)
        self.get_logger().info(f"Images will be saved to: {self.img_dest.absolute()}")
        
        self.count = 0
        self.current_frame = None
        self.lock = threading.Lock()
        self.save_requested = False
        self.active = True
        
        self.bridge = CvBridge()
        self.subscription = self.create_subscription(
            Image,
            '/image_raw',
            self.image_callback,
            10)
        
        # Display thread
        self.display_thread = threading.Thread(target=self.display_loop)
        self.display_thread.start()
        
        # Input thread (restored)
        self.input_thread = threading.Thread(target=self.input_loop)
        self.input_thread.start()
        
        self.get_logger().info("Press 's' in window or Enter in terminal to save, 'q' to quit")

    def image_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            with self.lock:
                self.current_frame = frame
        except Exception as e:
            self.get_logger().error(f"Image conversion error: {str(e)}")

    def display_loop(self):
        while self.active and rclpy.ok():
            with self.lock:
                if self.current_frame is not None:
                    display_frame = self.current_frame.copy()
                    
                    if self.save_requested:
                        cv2.putText(display_frame, "SAVING...", (10, 30),
                                   cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
                    
                    cv2.imshow('Camera Feed', display_frame)
            
            key = cv2.waitKey(20)
            if key == ord('q') or not self.active:
                break
            elif key == ord('s'):
                self.save_image()

    def input_loop(self):
        while self.active and rclpy.ok():
            user_input = input("Press Enter to save image or 'q' to quit: ")
            if user_input.lower() == 'q':
                self.active = False
                break
            else:
                self.save_image()

    def save_image(self):
        with self.lock:
            if self.current_frame is None:
                self.get_logger().warn("No frame available to save")
                return
                
            self.save_requested = True
            time.sleep(0.1)  # Let the display update
            
            try:
                dest = self.img_dest / f"img{self.count}.jpg"
                cv2.imwrite(str(dest), self.current_frame)
                self.get_logger().info(f"Image saved to: {dest}")
                self.count += 1
            except Exception as e:
                self.get_logger().error(f"Save failed: {str(e)}")
            finally:
                self.save_requested = False

    def destroy_node(self):
        self.get_logger().info("Shutting down...")
        self.active = False
        cv2.destroyAllWindows()
        if self.display_thread.is_alive():
            self.display_thread.join()
        if self.input_thread.is_alive():
            self.input_thread.join()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    
    parser = argparse.ArgumentParser()
    parser.add_argument("-o", "--output", required=True,
                      help="Output directory for images (will be created if doesn't exist)")
    args, _ = parser.parse_known_args()
    
    node = ImageSaver(args.output)
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()