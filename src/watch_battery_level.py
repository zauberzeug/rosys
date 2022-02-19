#!/usr/bin/env python3
from nicegui import ui
from rosys import event
from rosys.actors import Actor
from rosys.automations import drive_path
from rosys.world import PathSegment, Pose, Spline
import rosys.ui

runtime = rosys.Runtime()
rosys.ui.configure(ui, runtime)


class BatteryGuard(Actor):

    def __init__(self) -> None:
        super().__init__()
        event.register(event.Id.NEW_MACHINE_DATA, self.check_battery)

    def check_battery(self):
        if self.world.robot.battery < 24:
            event.emit(event.Id.PAUSE_AUTOMATION, 'battery level is below 24 V')


runtime.with_actors(BatteryGuard())
voltage = ui.label()
ui.timer(1, lambda: voltage.set_text(f'{runtime.world.robot.battery:.1f} V, pose: {runtime.world.robot.prediction}'))
path = [PathSegment(spline=Spline.from_poses(Pose(), Pose(x=10, y=2)))]


async def automation():
    await drive_path(runtime.world, runtime.hardware, path)
rosys.ui.automation_controls(automation)

ui.run(title='RoSys', port=8080)
