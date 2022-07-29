#!/usr/bin/env python3
import rosys
from nicegui import ui
from rosys.automation import AutomationControls, Automator
from rosys.driving import Driver, Odometer, PathSegment, RobotObject, RobotShape
from rosys.geometry import Pose, Spline
from rosys.hardware import WheelsSimulation


class GeoFenceGuard:

    def __init__(self, odometer: Odometer, automator: Automator) -> None:
        self.odometer = odometer
        self.automator = automator
        rosys.on_repeat(self.check_position, 0.1)

    def check_position(self) -> None:
        if abs(self.odometer.prediction.x) > 3 or abs(self.odometer.prediction.y) > 3:
            self.automator.pause('robot left the area')


# setup
async def automation() -> None:
    await driver.drive_path([PathSegment(spline=Spline.from_poses(Pose(), Pose(x=5, y=1)))])
shape = RobotShape()
odometer = Odometer()
wheels = WheelsSimulation(odometer)
driver = Driver(wheels, odometer)
automator = Automator(wheels, None, default_automation=automation)
geo_fence_guard = GeoFenceGuard(odometer, automator)

# ui
with ui.scene():
    RobotObject(shape, odometer)
label = ui.label()
ui.timer(0.1, lambda: label.set_text(f'pose: {odometer.prediction}'))
AutomationControls(automator)

# start
ui.run(title='RoSys')
