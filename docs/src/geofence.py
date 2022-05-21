#!/usr/bin/env python3
import rosys.ui
from nicegui import ui
from rosys import Runtime, event
from rosys.actors import Actor
from rosys.automations import drive_path
from rosys.world import PathSegment, Pose, Spline


class GeoFenceGuard(Actor):

    def __init__(self) -> None:
        super().__init__()
        event.register(event.Id.NEW_MACHINE_DATA, self.check_position)

    def check_position(self) -> None:
        if abs(self.world.robot.prediction.x) > 3 or abs(self.world.robot.prediction.y) > 3:
            event.emit(event.Id.PAUSE_AUTOMATION, 'robot left the area')


runtime = Runtime().with_actors(GeoFenceGuard())
rosys.ui.configure(ui, runtime)
label = ui.label()
ui.timer(0.1, lambda: label.set_text(f'pose: {runtime.world.robot.prediction}'))
path = [PathSegment(spline=Spline.from_poses(Pose(), Pose(x=5, y=1)))]


async def automation():
    await drive_path(runtime.world, runtime.hardware, path)
rosys.ui.automation_controls(default_automation=automation)

ui.run(title='RoSys')
