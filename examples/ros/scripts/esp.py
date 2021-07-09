#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist, Vector3
import serial
from operator import ixor
from functools import reduce
import json

def handle_command(data):

    line = f'drive speed {data.linear.x:3f},{data.angular.z:.3f}'
    line = f'{line}^{reduce(ixor, map(ord, line))}\n'
    port.write(line.encode())

if __name__ == '__main__':

    rospy.init_node('esp')

    publish_odometry = rospy.Publisher('/odometry', Twist, queue_size=1).publish
    publish_status = rospy.Publisher('/status', String, queue_size=1).publish

    rospy.Subscriber('/command', Twist, handle_command, queue_size=1)

    with serial.Serial('/dev/esp', 115200) as port:

        while not rospy.is_shutdown():
            try:
                line = port.readline().decode()
            except UnicodeDecodeError:
                continue

            if '^' in line:
                line, checksum = line.split('^')
                if reduce(ixor, map(ord, line)) != int(checksum):
                    continue

            words = line.split()
            if not any(words):
                continue
            if words.pop(0) != 'esp':
                continue
            time = int(words.pop(0))
            linear_speed = float(words.pop(0))
            angular_speed = float(words.pop(0))
            temperature = float(words.pop(0))
            battery = float(words.pop(0))
            state = words.pop(0).lower()

            publish_odometry(Twist(
                Vector3(linear_speed, 0, 0),
                Vector3(0, 0, angular_speed),
            ))
            publish_status(json.dumps({
                'time': time,
                'temperature': temperature,
                'battery': battery,
                'state': state,
            }))
