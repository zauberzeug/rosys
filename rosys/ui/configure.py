from nicegui.ui import Ui
from .. import Runtime
from .. import event
from .joystick import Joystick
from .keyboard_control import KeyboardControl
from .robot_object import RobotObject
from .automation_controls import AutomationControls
from .lizard_serial_debug import LizardSerialDebug
from .objgraph_page import ObjgraphPage
from .pyloot_page import PylootPage
from . import routes


def configure(ui: Ui, runtime: Runtime):
    ui.on_startup(runtime.startup())
    ui.on_shutdown(runtime.shutdown())
    event.register(event.Id.NEW_NOTIFICATION, ui.notify)
    routes.setup(ui, runtime)

    Joystick.steerer = runtime.steerer
    KeyboardControl.ui = ui
    KeyboardControl.steerer = runtime.steerer
    RobotObject.robot = runtime.world.robot
    RobotObject.ui = ui
    AutomationControls.runtime = runtime
    AutomationControls.ui = ui
    LizardSerialDebug.ui = ui
    LizardSerialDebug.communication = runtime.hardware.communication
    ObjgraphPage.ui = ui
    PylootPage.ui = ui
