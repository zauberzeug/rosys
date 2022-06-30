from . import actors, event, hardware, run, task_logger, world
from .automation import Automation
from .core import core
from .lifecycle import on_repeat, on_shutdown, on_startup, shutdown, startup
from .persistence import Persistence
from .runtime import Runtime

notify = core.notify
sleep = core.sleep
is_test = core.is_test


def time() -> float:
    return core.time
