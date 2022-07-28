#!/usr/bin/env python3
import rosys.ui
from nicegui import ui
from rosys.actors import Automator, Driver, Odometer
from rosys.hardware import WheelsSimulation
from rosys.world import PathSegment, Pose, RobotShape, Spline


class GeoFenceGuard:

    def __init__(self, odometer: Odometer, automator: Automator) -> None:
        self.odometer = odometer
        self.automator = automator
        rosys.on_repeat(self.check_position, 0.1)

    def check_position(self) -> None:
        if abs(self.odometer.prediction.x) > 3 or abs(self.odometer.prediction.y) > 3:
            self.automator.PAUSE_AUTOMATION.emit('robot left the area')


# setup
shape = RobotShape()
odometer = Odometer()
wheels = WheelsSimulation(odometer)
driver = Driver(wheels, odometer)
automator = Automator()
geo_fence_guard = GeoFenceGuard(odometer, automator)

# ui
rosys.NEW_NOTIFICATION.register(ui.notify)
with ui.scene():
    rosys.ui.robot_object(shape, odometer)
label = ui.label()
ui.timer(0.1, lambda: label.set_text(f'pose: {odometer.prediction}'))


async def automation():
    await driver.drive_path([PathSegment(spline=Spline.from_poses(Pose(), Pose(x=5, y=1)))])
rosys.ui.automation_controls(automator, default_automation=automation)

# start
ui.on_startup(rosys.startup)
ui.on_shutdown(rosys.shutdown)
ui.run(title='RoSys')
