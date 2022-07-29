#!/usr/bin/env python3
from nicegui import ui
from rosys.automation import AutomationControls, Automator
from rosys.driving import Driver, Odometer, RobotObject, RobotShape
from rosys.geometry import Point
from rosys.hardware import WheelsSimulation

# setup
shape = RobotShape()
odometer = Odometer()
wheels = WheelsSimulation(odometer)
driver = Driver(wheels, odometer)
automator = Automator(wheels, None)


async def handle_click(msg):
    for hit in msg.hits:
        target = Point(x=hit.point.x, y=hit.point.y)
        automator.start(driver.drive_to(target))

# ui
with ui.scene(on_click=handle_click):
    RobotObject(shape, odometer, debug=True)
ui.label('click into the scene to drive the robot')
with ui.row():
    AutomationControls(automator)
ui.label('you can also pause/resume or stop the running automation')

# start
ui.run(title='RoSys')
