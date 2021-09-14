#!/usr/bin/env python3
import rospy
from std_msgs.msg import Empty, String
from geometry_msgs.msg import Twist, Vector3
import serial
from operator import ixor
from functools import reduce
import json
import os.path


def send(line):
    line = f'{line}^{reduce(ixor, map(ord, line))}\n'
    port.write(line.encode())


def handle_command(data):
    send(f'drive speed {data.linear.x:3f},{data.angular.z:.3f}')


def handle_configure(data):
    with open(os.path.dirname(__file__) + '/../config.txt') as f:
        send('esp erase')
        for line in f.read().splitlines():
            line = line.strip()
            if line and not line.startswith('#'):
                send('+' + line)
        send('esp restart')


if __name__ == '__main__':
    rospy.init_node('esp')
    publish_odometry = rospy.Publisher('/odometry', Twist, queue_size=1).publish
    publish_status = rospy.Publisher('/status', String, queue_size=1).publish

    rospy.Subscriber('/steer', Twist, handle_command, queue_size=1)
    rospy.Subscriber('/configure', Empty, handle_configure, queue_size=1)

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
