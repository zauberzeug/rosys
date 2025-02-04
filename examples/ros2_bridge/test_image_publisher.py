#!/usr/bin/env python3

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from builtin_interfaces.msg import Time


class TestImagePublisher(Node):
    def __init__(self):
        super().__init__('test_image_publisher')
        self.publisher = self.create_publisher(Image, 'test_image', 10)
        self.bridge = CvBridge()
        self.timer = self.create_timer(1.0, self.publish_image)  # 1 Hz

    def publish_image(self):
        # Create a test image (a simple gradient)
        height, width = 480, 640
        image = np.zeros((height, width, 3), dtype=np.uint8)

        # Create a color gradient
        for i in range(height):
            for j in range(width):
                image[i, j] = [
                    int(255 * i / height),
                    int(255 * j / width),
                    128
                ]

        # Convert the image to a ROS message
        ros_image = self.bridge.cv2_to_imgmsg(image, encoding="bgr8")
        
        # Set current timestamp
        now = self.get_clock().now()
        ros_image.header.stamp = now.to_msg()

        # Publish the image
        self.publisher.publish(ros_image)
        
        # Calculate timestamp in seconds for logging
        timestamp = now.seconds_nanoseconds()
        ts = timestamp[0] + timestamp[1] / 1e9
        self.get_logger().info(f"Published test image at time {ts:.9f}")


def main(args=None):
    rclpy.init(args=args)
    publisher = TestImagePublisher()
    try:
        rclpy.spin(publisher)
    except KeyboardInterrupt:
        pass
    finally:
        publisher.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main() 