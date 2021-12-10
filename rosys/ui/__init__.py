from nicegui.ui import Ui
from nicegui import app
from ..runtime import Runtime
from .joystick import Joystick as joystick
from .keyboard_control import KeyboardControl as keyboard_control
from .obstacle_object import ObstacleObject as obstacle_object
from .path_object import PathObject as path_object
from .robot_object import RobotObject as robot_object
from .automation_controls import AutomationControls as automation_controls
from .lizard_serial_debug import LizardSerialDebug as lizard_serial_debug
from .objgraph_page import ObjgraphPage as objgraph_page
from .pyloot_page import PylootPage as pyloot_page
from .. import event


def configure(ui: Ui, runtime: Runtime):
    ui.on_startup(runtime.startup())
    ui.on_shutdown(runtime.shutdown())
    event.register(event.Id.NEW_NOTIFICATION, ui.notify)

    joystick.steerer = runtime.steerer
    keyboard_control.ui = ui
    keyboard_control.steerer = runtime.steerer
    robot_object.robot = runtime.world.robot
    robot_object.ui = ui
    automation_controls.runtime = runtime
    automation_controls.ui = ui
    lizard_serial_debug.ui = ui
    lizard_serial_debug.communication = runtime.hardware.communication
    objgraph_page.ui = ui
    pyloot_page.ui = ui
    pyloot_page.app = app
