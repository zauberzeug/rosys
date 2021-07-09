#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist, Vector3
from nicegui import ui

rospy.init_node('ui')

publish = rospy.Publisher('/command', Twist, queue_size=1).publish

def send(x, y):
    linear = Vector3(x, 0, 0)
    angular = Vector3(0, 0, y)
    publish(Twist(linear, angular))

ui.joystick(
    color='blue',
    size=50,
    on_move=lambda msg: send(msg.data.vector.x, msg.data.vector.y),
    on_end=lambda _: send(0, 0))

ui.run(reload=False, show=False)
