from pydantic import BaseModel, PrivateAttr
from aenum import Enum, auto
import time
from .mode import Mode
from .obstacle import Obstacle
from .robot import Robot


class AutomationState(str, Enum, init='value __doc__'):

    def _generate_next_value_(name, start, count, last_values):
        '''uses enum name as value when calling auto()'''
        return name

    DISABLED = auto(), 'no automations available or execution not allowed'
    STOPPED = auto(), 'there is an automation which could be started'
    RUNNING = auto(), 'automations are beeing processed'
    PAUSED = auto(), 'an ongoing automation can be resumed'


class World(BaseModel):

    robot: Robot = Robot()
    mode: Mode = Mode.REAL
    automation_state: AutomationState = AutomationState.DISABLED
    _time: float = PrivateAttr(default_factory=time.time)
    tracking: bool = False
    robot_locator_cam: str = None
    download_queue: list[str] = []
    image_data: dict[str, bytes] = {}
    link_queue: list[list[str]] = []
    obstacles: dict[str, Obstacle] = {}
    notifications: list[tuple[float, str]] = []

    @property
    def time(self):
        return self._time if self.mode == Mode.TEST else time.time()

    def set_time(self, value):
        assert self.mode == Mode.TEST
        self._time = value
