#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
import serial

def handle_command(data):

    print('command: %.3f, %.3f' % (data.linear.x, data.angular.z))

if __name__ == '__main__':

    rospy.init_node('esp')

    rospy.Subscriber('/command', Twist, handle_command, queue_size=1)

    with serial.Serial('/dev/serial', 115200) as port:

        while not rospy.is_shutdown():
            line = port.readline()
            print(line)
