#!/usr/bin/env python3
import random

from nicegui import app, ui

import rosys
from rosys.automation import Automator
from rosys.driving import Driver, Odometer, robot_object
from rosys.geometry import Point, Prism
from rosys.hardware import WheelsSimulation

wheels = WheelsSimulation()
odometer = Odometer(wheels)
driver = Driver(wheels, odometer)
driver.parameters.linear_speed_limit = 3
driver.parameters.angular_speed_limit = 1
automator = Automator(wheels, None)

size = 3
boundary = [(-size, -size), (-size, size), (size, size), (size, -size)]

with ui.scene() as scene:
    robot_object(Prism.default_robot_shape(), odometer)
    for i, a in enumerate(boundary):
        b = boundary[(i+1) % len(boundary)]
        ui.scene.line([*a, 0.1], [*b, 0.1]).material('red')
    scene.move_camera(0, 0, 8)
with ui.column().style('width: 400px'):
    rosys.simulation_ui()


async def move_around():
    while True:
        await driver.drive_to(Point(x=random.uniform(-size, size), y=random.uniform(-size, size)))

app.on_startup(lambda: automator.start(move_around()))

ui.run(title='RoSys')
