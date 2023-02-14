import abc
import logging
from typing import Optional, cast

from .. import rosys
from .module import Module, ModuleHardware, ModuleSimulation
from .robot_brain import RobotBrain


class Robot(abc.ABC):

    def __init__(self, modules: list[Module]) -> None:
        self.log = logging.getLogger(__name__)
        self.modules = modules


class RobotHardware(Robot):

    def __init__(self, modules: list[Module], robot_brain: RobotBrain) -> None:
        super().__init__(modules)

        self.robot_brain = robot_brain
        rosys.on_repeat(self.update, 0.01)

    async def configure(self) -> None:
        startup_code = ''
        for module in self.modules:
            startup_code += cast(ModuleHardware, module).lizard_code + '\n'
        output_fields = []
        for module in self.modules:
            output_fields.extend(cast(ModuleHardware, module).CORE_MESSAGE_FIELDS)
        startup_code += f'''
            core.output("core.millis {' '.join(output_fields)}")
            en = Output(15)
            v24 = Output(12)
            en.on()
            v24.on()
        '''
        await self.robot_brain.configure(startup_code)

    async def update(self) -> None:
        for time, line in await self.robot_brain.read_lines():
            words = line.split()
            if words[0] == 'core':
                words.pop(0)
                words.pop(0)
                for module in self.modules:
                    await cast(ModuleHardware, module).handle_core_output(time, words)
            else:
                for module in self.modules:
                    if words[0] in cast(ModuleHardware, module).message_hooks:
                        cast(ModuleHardware, module).message_hooks[words[0]](time, words)


class RobotSimulation(Robot):

    def __init__(self, modules: list[Module]) -> None:
        super().__init__(modules)
        self._last_step: Optional[float] = None
        rosys.on_repeat(self.step, 0.01)

    async def step(self) -> None:
        if self._last_step is not None:
            dt = rosys.time() - self._last_step
            for module in self.modules:
                await cast(ModuleSimulation, module).step(dt)
        self._last_step = rosys.time()
