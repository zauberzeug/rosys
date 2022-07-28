#!/usr/bin/env python3
import rosys.ui
from nicegui import ui
import rosys
from rosys.actors import Automator, Driver, Odometer
from rosys.hardware import WheelsSimulation
from rosys.world import Point, RobotShape

# setup
shape = RobotShape()
odometer = Odometer()
wheels = WheelsSimulation(odometer)
driver = Driver(wheels, odometer)
automator = Automator()


async def handle_click(msg):
    for hit in msg.hits:
        target = Point(x=hit.point.x, y=hit.point.y)
        automator.start(driver.drive_to(target))

# ui
rosys.NEW_NOTIFICATION.register(ui.notify)
with ui.scene(on_click=handle_click):
    rosys.ui.robot_object(shape, odometer, debug=True)
ui.label('click into the scene to drive the robot')
with ui.row():
    rosys.ui.automation_controls(automator)
ui.label('you can also pause/resume or stop the running automation')

# start
ui.on_startup(rosys.startup)
ui.on_shutdown(rosys.shutdown)
ui.run(title='RoSys')
