#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
import serial

def handle_command(data):

    rospy.logwarn('%.3f, %.3f', data.linear.x, data.angular.z)

if __name__ == '__main__':

    rospy.init_node('serial')

    rospy.Subscriber('/command', Twist, handle_command, queue_size=1)

    rospy.spin()
