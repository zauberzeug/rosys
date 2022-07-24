#!/usr/bin/env python3
import rosys.ui
from nicegui import ui
from rosys import runtime
from rosys.actors import Automator, Driver, Odometer
from rosys.hardware import WheelsSimulation
from rosys.world import Point, Robot

# setup
robot = Robot()
odometer = Odometer()
wheels = WheelsSimulation(odometer)
driver = Driver(wheels)
automator = Automator()


async def drive():
    await driver.drive_to(Point(x=1, y=0))
    await driver.drive_to(Point(x=0, y=0))

# ui
ui.label('define at which times the automations should run')
rosys.ui.time_schedule(automator)
runtime.NEW_NOTIFICATION.register(ui.notify)
with ui.scene():
    rosys.ui.robot_object(robot, odometer)
with ui.row():
    rosys.ui.automation_controls(automator, default_automation=drive)

# start
ui.on_startup(runtime.startup)
ui.on_shutdown(runtime.shutdown)
ui.run(title='RoSys')
