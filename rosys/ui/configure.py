from nicegui.ui import Ui
from rosys.ui.camera_objects import CameraObjects

from .. import Runtime, event
from ..hardware import CommunicatingHardware
from . import routes
from .asyncio_page import AsyncioPage
from .automation_controls import AutomationControls
from .cpu_usage import CpuUsage
from .joystick import Joystick
from .keyboard_control import KeyboardControl
from .lizard_serial_debug import LizardSerialDebug
from .lizard_stats import LizardStats
from .objgraph_page import ObjgraphPage
from .pyloot_page import PylootPage
from .robot_object import RobotObject


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
    if isinstance(runtime.hardware, CommunicatingHardware):
        LizardSerialDebug.communication = runtime.hardware.communication
    ObjgraphPage.ui = ui
    PylootPage.ui = ui
    CpuUsage.ui = ui
    LizardStats.ui = ui
    LizardStats.lizard = runtime.lizard
    AsyncioPage.ui = ui
    AsyncioPage.asyncio_monitor = runtime.asyncio_monitor
    CameraObjects.world = runtime.world
