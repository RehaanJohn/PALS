#!/usr/bin/env python3
# ==============================================================================
# stream_video.py
# Streams the /camera topic from ROS 2 to QGroundControl via UDP (h.264)
# ==============================================================================

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import sys

class VideoStreamer(Node):
    def __init__(self):
        super().__init__('qgc_video_streamer')
        self.get_logger().info("Initializing QGC Video Streamer...")
        
        self.subscription = self.create_subscription(
            Image,
            '/camera',
            self.image_callback,
            10)
        self.bridge = CvBridge()
        
        # QGroundControl listens for H.264 video on UDP port 5600 by default
        pipeline = (
            "appsrc ! videoconvert ! video/x-raw,format=I420 ! "
            "x264enc tune=zerolatency bitrate=1000 speed-preset=superfast ! "
            "rtph264pay ! udpsink host=127.0.0.1 port=5600"
        )
        
        self.out = cv2.VideoWriter(pipeline, cv2.CAP_GSTREAMER, 0, 20.0, (640, 480))
        
        if not self.out.isOpened():
            self.get_logger().error("Failed to open GStreamer pipeline. Ensure gstreamer is installed.")
            sys.exit(1)
            
        self.get_logger().info("Streaming /camera to QGroundControl at 127.0.0.1:5600")

    def image_callback(self, data):
        try:
            # Convert ROS Image message to OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            # Write to GStreamer pipeline
            self.out.write(cv_image)
        except Exception as e:
            self.get_logger().error(f"Error processing image: {e}")

def main(args=None):
    rclpy.init(args=args)
    try:
        streamer = VideoStreamer()
        rclpy.spin(streamer)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()
