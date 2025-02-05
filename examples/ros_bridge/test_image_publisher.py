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
    rate = rospy.Rate(30)

    while not rospy.is_shutdown():
        # Create a test image (a simple gradient)
        height, width = 1080, 1920
        image = np.zeros((height, width, 3), dtype=np.uint8)

        # Get current timestamp
        timestamp = rospy.get_time()

        # Fill entire image with blue color
        image[:, :] = [255, 0, 0]  # BGR format - blue is [0,0,255]

        # Write timestamp on the image
        timestamp_text = f"Time: {timestamp:.2f}"
        cv2.putText(image, timestamp_text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX,
                    1, (255, 255, 255), 2, cv2.LINE_AA)

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
