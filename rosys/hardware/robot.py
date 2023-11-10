import abc
import logging
from typing import Optional, cast

from .. import rosys
from ..helpers import remove_indentation
from .expander import ExpanderHardware
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
        self.robot_brain.lizard_code = self.generate_lizard_code()
        self.expander_prefixes = set(f'{module.name}:' for module in modules if isinstance(module, ExpanderHardware))
        rosys.on_repeat(self.update, 0.01)

    def generate_lizard_code(self) -> str:
        code = remove_indentation('''
            rdyp = Output(15)
            en3 = Output(12)
        ''')
        for module in self.modules:
            code += cast(ModuleHardware, module).lizard_code + '\n'
        output_fields = []
        for module in self.modules:
            output_fields.extend(cast(ModuleHardware, module).core_message_fields)
        code += remove_indentation(f'''
            core.output("core.millis {' '.join(output_fields)}")
            rdyp.on()
            en3.on()
        ''')
        return code

    async def update(self) -> None:
        for time, line in await self.robot_brain.read_lines():
            words = line.split()
            if not words:
                continue
            if words[0] in self.expander_prefixes:
                words.pop(0)
            if not words:
                continue
            if words[0] == 'core':
                words.pop(0)
                words.pop(0)
                for module in self.modules:
                    cast(ModuleHardware, module).handle_core_output(time, words)
            else:
                for module in self.modules:
                    if words[0] in cast(ModuleHardware, module).message_hooks:
                        cast(ModuleHardware, module).message_hooks[words[0]](line)


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
