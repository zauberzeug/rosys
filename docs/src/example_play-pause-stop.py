#!/usr/bin/env python3
from nicegui import ui
from rosys.automation import Automator, automation_controls
from rosys.driving import Driver, Odometer, robot_object
from rosys.geometry import Point, Prism
from rosys.hardware import WheelsSimulation


async def run() -> None:
    for c in checkpoints:
        await driver.drive_to(c)

checkpoints: list[Point] = [Point(x=-3, y=1), Point(x=3, y=3), Point(x=2, y=-2)]
wheels = WheelsSimulation()
odometer = Odometer(wheels)
driver = Driver(wheels, odometer)
automator = Automator(wheels)
automator.default_automation = run

with ui.scene(width=600) as scene:
    robot_object(Prism.default_robot_shape(), odometer)
    for n, c in enumerate(checkpoints):
        scene.text(f'{n+1}').move(x=c.x, y=c.y)
with ui.row():
    automation_controls(automator)

ui.run(title='RoSys')
