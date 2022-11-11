#!/usr/bin/env python3
from nicegui import ui

from rosys.automation import Automator, automation_controls
from rosys.driving import Driver, Odometer, robot_object
from rosys.geometry import Point, Prism
from rosys.hardware import WheelsSimulation

shape = Prism.default_robot_shape()
wheels = WheelsSimulation()
odometer = Odometer(wheels)
driver = Driver(wheels, odometer)
automator = Automator(wheels, None)


async def handle_click(msg):
    for hit in msg.hits:
        if hit.object_id == 'ground':
            target = Point(x=hit.point.x, y=hit.point.y)
            automator.start(driver.drive_to(target))

with ui.scene(on_click=handle_click):
    robot_object(shape, odometer, debug=True)
ui.label('click into the scene to drive the robot')
with ui.row():
    automation_controls(automator)
ui.label('you can also pause/resume or stop the running automation')

ui.run(title='RoSys')
