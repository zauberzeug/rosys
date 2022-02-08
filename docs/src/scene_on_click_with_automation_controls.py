#!/usr/bin/env python3
from nicegui import ui
import rosys
import rosys.ui
from rosys.automations import drive_to
from rosys.world import Point

runtime = rosys.Runtime()
rosys.ui.configure(ui, runtime)


async def handle_click(msg):
    for hit in msg.hits:
        target = Point(x=hit.point.x, y=hit.point.y)
        await runtime.automator.start(drive_to(runtime.world, runtime.hardware, target))


with ui.scene(on_click=handle_click) as scene:
    robot = rosys.ui.robot_object(debug=True)
ui.label('click into the scene to drive the robot')
with ui.row():
    rosys.ui.automation_controls()
ui.label('you can also pause/resume or stop the running automation')

ui.run(title='RoSys', port=8080)
