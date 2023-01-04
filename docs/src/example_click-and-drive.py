#!/usr/bin/env python3
from nicegui import ui
from nicegui.events import SceneClickEventArguments

import rosys

wheels = rosys.hardware.WheelsSimulation()
odometer = rosys.driving.Odometer(wheels)
driver = rosys.driving.Driver(wheels, odometer)
automator = rosys.automation.Automator(wheels, None)


async def handle_click(e: SceneClickEventArguments):
    for hit in e.hits:
        if hit.object_id == 'ground':
            target = rosys.geometry.Point(x=hit.x, y=hit.y)
            automator.start(driver.drive_to(target))

with ui.scene(on_click=handle_click):
    shape = rosys.geometry.Prism.default_robot_shape()
    rosys.driving.robot_object(shape, odometer, debug=True)
ui.label('click into the scene to drive the robot')
with ui.row():
    rosys.automation.automation_controls(automator)
ui.label('you can also pause/resume or stop the running automation')

ui.run(title='RoSys')
