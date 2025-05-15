#!/usr/bin/env python3
from nicegui import ui

import rosys
from rosys.geometry import Point, Prism


async def run() -> None:
    for c in checkpoints:
        await driver.drive_to(c)

checkpoints: list[Point] = [Point(x=-3, y=1), Point(x=3, y=3), Point(x=2, y=-2)]
wheels = rosys.hardware.WheelsSimulation()
robot = rosys.hardware.RobotSimulation([wheels])
odometer = rosys.driving.Odometer(wheels)
driver = rosys.driving.Driver(wheels, odometer)
automator = rosys.automation.Automator(None, default_automation=run,
                                       on_interrupt=wheels.stop)

with ui.scene(width=600).classes('drop-shadow-lg') as scene:
    rosys.driving.robot_object(Prism.default_robot_shape(), odometer)
    for i, point in enumerate(checkpoints):
        scene.text(f'{i+1}').move(x=point.x, y=point.y)

with ui.row():
    rosys.automation.automation_controls(automator)

ui.run(title='RoSys')
