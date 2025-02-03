#!/usr/bin/env python3

import cv2
import numpy as np
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image


def publish_test_image():
    # Initialize the ROS node
    rospy.init_node('test_image_publisher')

    # Create a publisher for the image topic
    pub = rospy.Publisher('test_image', Image, queue_size=10)

    # Create a CV bridge
    bridge = CvBridge()

    # Set the publishing rate (1 Hz)
    rate = rospy.Rate(1)

    while not rospy.is_shutdown():
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
        ros_image = bridge.cv2_to_imgmsg(image, encoding="bgr8")

        # Publish the image
        pub.publish(ros_image)
        rospy.loginfo("Published test image")

        rate.sleep()


if __name__ == '__main__':
    try:
        publish_test_image()
    except rospy.ROSInterruptException:
        pass
