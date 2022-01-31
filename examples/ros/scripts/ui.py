#!/usr/bin/env python3
import rospy
from std_msgs.msg import Empty, String
from geometry_msgs.msg import Twist, Vector3
from nicegui import ui
import json


def send(x, y):
    linear = Vector3(y, 0, 0)
    angular = Vector3(0, 0, -x)
    publish_twist(Twist(linear, angular))


def handle_status(data):
    msg = json.loads(data.data)
    time.text = f'Timestamp: {msg["time"]} ms'


def handle_odometry(data):
    linear.value = data.linear.x
    angular.value = data.angular.z


rospy.init_node('ui')

publish_twist = rospy.Publisher('/steer', Twist, queue_size=1).publish
publish_configure = rospy.Publisher('/configure', Empty, queue_size=1).publish

with ui.row().classes('items-stretch'):
    with ui.card():
        ui.markdown('### Setup')
        ui.button('Configure', on_click=lambda: publish_configure())
    with ui.card() as status:
        ui.markdown('### Status')
        time = ui.label()
    with ui.card():
        ui.markdown('### Control')
        ui.joystick(
            color='blue',
            size=50,
            on_move=lambda msg: send(msg.data.vector.x, msg.data.vector.y),
            on_end=lambda _: send(0, 0),
        )
    with ui.card() as odometry:
        ui.markdown('### Odometry')
        ui.label('linear')
        linear = ui.slider(min=-1.0, max=1.0, step=0.01).props('label-always readonly')
        ui.label('angular')
        angular = ui.slider(min=-1.0, max=1.0, step=0.01).props('label-always readonly')

    ui.timer(0.1, lambda: None)  # NOTE: update ui

rospy.Subscriber('/status', String, handle_status, queue_size=1)
rospy.Subscriber('/odometry', Twist, handle_odometry, queue_size=1)

ui.run(title='ROS Example', reload=False, show=False)
