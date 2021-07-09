#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist, Vector3
from nicegui import ui
import json

def send(x, y):

    linear = Vector3(y, 0, 0)
    angular = Vector3(0, 0, -x)
    publish(Twist(linear, angular))

def handle_status(data):

    msg = json.loads(data.data)
    time.text = f'Timestamp: {msg["time"]} ms'
    temperature.text = f'Temperature: {msg["temperature"]} C'
    battery.text = f'Battery: {msg["battery"]} V'
    state.text = f'State: {msg["state"]}'

def handle_odometry(data):

    linear.value = data.linear.x
    angular.value = data.angular.z

with ui.row():

    with ui.card() as status:
        ui.markdown('### Status')
        time = ui.label()
        temperature = ui.label()
        battery = ui.label()
        state = ui.label()

    with ui.card() as odometry:
        ui.markdown('### Odometry')
        ui.label('linear')
        linear = ui.slider(min=-1.0, max=1.0, step=0.01).props('label-always readonly')
        ui.label('angular')
        angular = ui.slider(min=-1.0, max=1.0, step=0.01).props('label-always readonly')

    with ui.card():
        ui.markdown('### Control')
        ui.joystick(
            color='blue',
            size=50,
            on_move=lambda msg: send(msg.data.vector.x, msg.data.vector.y),
            on_end=lambda _: send(0, 0),
        )

    ui.timer(0.1, lambda: None) # NOTE: update ui


rospy.init_node('ui')

publish = rospy.Publisher('/command', Twist, queue_size=1).publish

rospy.Subscriber('/status', String, handle_status, queue_size=1)
rospy.Subscriber('/odometry', Twist, handle_odometry, queue_size=1)


ui.run(reload=False, show=False)
