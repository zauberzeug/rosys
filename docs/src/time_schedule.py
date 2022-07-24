#!/usr/bin/env python3
from datetime import datetime

import rosys.ui
from nicegui import ui
from rosys import runtime
from rosys.actors import Automator, Driver, Odometer, TimeSchedule
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

time_schedule = TimeSchedule(automator, automation=drive)
time_schedule.plan.disable_all()

# ui
now = str(datetime.now().hour).zfill(2)
ui.label(f'you can test the time schedule by clicking on the "{now}" slot; '
         f'this will restart the automation for the current hour')
time_schedule.ui()
runtime.NEW_NOTIFICATION.register(ui.notify)
with ui.scene():
    rosys.ui.robot_object(robot, odometer)
with ui.row():
    rosys.ui.automation_controls(automator, default_automation=drive)

# start
ui.on_startup(runtime.startup)
ui.on_shutdown(runtime.shutdown)
ui.run(title='RoSys')
