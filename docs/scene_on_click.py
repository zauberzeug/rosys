#!/usr/bin/env python3
from nicegui import ui
import rosys
import rosys.ui
from rosys.automations import drive_to

runtime = rosys.Runtime()
rosys.ui.configure(ui, runtime)


def handle_click(msg):
    for hit in msg.hits:
        target = rosys.Point(x=hit.point.x, y=hit.point.y)
        runtime.automator.replace(drive_to(runtime.world, runtime.esp, target))
        runtime.resume()


with ui.scene(on_click=handle_click) as scene:
    robot = rosys.ui.robot_object(debug=True)
    ui.timer(0.05, robot.update)
ui.label('klick into the scene to drive the robot')

ui.run(title="RoSys", port=8080)
