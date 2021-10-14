from .actors.esp import Esp
from .actors.serial_esp import SerialEsp
from .actors.mocked_esp import MockedEsp
from .actors.web_esp import WebEsp
from .world.world import World
import logging

log = logging.getLogger(__name__)


def create_esp(world: World) -> Esp:
    try:
        return SerialEsp()
    except:
        log.warning('could not create serial esp')
        pass
    try:
        return WebEsp(world)
    except:
        log.warning('could not create web esp')
        pass
    return MockedEsp(world)
