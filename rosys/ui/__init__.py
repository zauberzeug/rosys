from nicegui.ui import Ui
from ..runtime import Runtime
from .joystick import Joystick as joystick
from .keyboard_control import KeyboardControl as keyboard_control
from .obstacle_object import ObstacleObject as obstacle_object
from .path_object import PathObject as path_object
from .robot_object import RobotObject as robot_object
from .automation_controls import AutomationControls as automation_controls
from .. import event


def configure(ui: Ui, runtime: Runtime):

    ui.on_startup(runtime.start())
    ui.on_shutdown(runtime.stop())
    event.register(event.Id.NEW_NOTIFICATION, ui.notify)

    joystick.steerer = runtime.steerer
    keyboard_control.ui = ui
    keyboard_control.steerer = runtime.steerer
    path_object.world = runtime.world
    robot_object.robot = runtime.world.robot
    robot_object.ui = ui
    automation_controls.runtime = runtime
    automation_controls.ui = ui
