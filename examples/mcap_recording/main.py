#!/usr/bin/env python3
from nicegui import ui

from rosys.analysis import recording
from rosys.analysis.recording import McapRecorder, RecordingsPage
from rosys.driving import Odometer, Steerer, keyboard_control, robot_object
from rosys.geometry import Prism
from rosys.hardware import RobotSimulation, WheelsSimulation

shape = Prism.default_robot_shape()
wheels = WheelsSimulation()
robot = RobotSimulation([wheels])
odometer = Odometer(wheels)
steerer = Steerer(wheels)

recorder = McapRecorder(auto_start=False)
recording.add_event_topic(recorder, '/wheels', event=wheels.VELOCITY_MEASURED, unpack=True)
recording.add_pose_topic(recorder, '/odometry/pose', event=odometer.POSE_UPDATED, child='odometry')

RecordingsPage(recorder, header=lambda: ui.link('steering', '/'))


@ui.page('/')
def index() -> None:
    keyboard_control(steerer)
    with ui.scene():
        robot_object(shape, odometer)
    recorder.developer_ui()


ui.run(title='MCAP Recording')
