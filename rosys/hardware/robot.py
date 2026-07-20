import abc
import logging
from typing import cast

from .. import rosys
from .expander import ExpanderHardware
from .module import Module, ModuleHardware, ModuleSimulation
from .robot_brain import RobotBrain


class Robot(abc.ABC):
    """A robot that consists of a number of modules.

    It can be either a hardware robot or a simulation.
    """

    def __init__(self, modules: list[Module]) -> None:
        self.log = logging.getLogger(__name__)
        self.modules = modules

    def add_module(self, module: Module) -> None:
        self.modules.append(module)


class RobotHardware(Robot):
    """A robot that consists of hardware modules.

    It generates Lizard code, forwards output to the hardware modules and sends commands to the robot brain.
    """

    def __init__(self, modules: list[Module], robot_brain: RobotBrain, *,
                 rdyp_pin: int | None = 15, en3_pin: int | None = 12) -> None:
        """Assemble the hardware robot from its modules and generate the Lizard code for the Robot Brain.

        :param rdyp_pin: pin of the RDYP enable output, or ``None`` to omit it (default: 15).
        :param en3_pin: pin of the EN3 enable output, or ``None`` to omit it (default: 12).

        Both default to the standard Robot Brain pins; override them (or pass ``None``) for robots wired differently.
        """
        super().__init__(modules)
        self.robot_brain = robot_brain
        self._rdyp_pin = rdyp_pin
        self._en3_pin = en3_pin
        self._expected_output_length = 0
        self._warning_cooldown = 30.0
        self._last_warning_time = 0.0
        self.robot_brain.lizard_code = self.generate_lizard_code()
        self.expander_prefixes = set(f'{module.name}:' for module in modules if isinstance(module, ExpanderHardware))
        rosys.on_repeat(self.update, 0.01, weak=True)

    def add_module(self, module: Module) -> None:
        super().add_module(module)
        self.robot_brain.lizard_code = self.generate_lizard_code()

    def generate_lizard_code(self) -> str:
        code = ''
        if self._rdyp_pin is not None:
            code += f'rdyp = Output({self._rdyp_pin})\n'
        if self._en3_pin is not None:
            code += f'en3 = Output({self._en3_pin})\n'
        for module in self.modules:
            code += cast(ModuleHardware, module).lizard_code + '\n'
        output_fields = []
        for module in self.modules:
            output_fields.extend(cast(ModuleHardware, module).core_message_fields)
        fields = ' '.join(output_fields)
        code += f'core.output("core.millis {fields}")\n'
        if self._rdyp_pin is not None:
            code += 'rdyp.on()\n'
        if self._en3_pin is not None:
            code += 'en3.on()\n'
        self._expected_output_length = len(output_fields) + 2  # account for the two words "core" and `core.millis`
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
                if len(words) != self._expected_output_length:
                    current_time = rosys.time()
                    if current_time - self._last_warning_time >= self._warning_cooldown:
                        rosys.notify('Lizard configuration is incorrect, '
                                     f'expected {self._expected_output_length} words, got {len(words)}',
                                     type='negative',
                                     log_level=logging.ERROR)
                        self._last_warning_time = current_time
                    return
                words.pop(0)
                words.pop(0)
                for module in self.modules:
                    cast(ModuleHardware, module).handle_core_output(time, words)
            else:
                for module in self.modules:
                    if words[0] in cast(ModuleHardware, module).message_hooks:
                        cast(ModuleHardware, module).message_hooks[words[0]](line)

    async def en3_on(self) -> None:
        """Release the EN3 software emergency stop to provide power to the robot."""
        await self.robot_brain.send('en3.on()')

    async def en3_off(self) -> None:
        """Activate the EN3 software emergency stop to cut power to the robot."""
        await self.robot_brain.send('en3.off()')

    async def rdyp_on(self) -> None:
        """Activate RDYP to provide power to the robot's hardware.

        Power will only be available if EN1, EN2 or EN3 are active.
        """
        await self.robot_brain.send('rdyp.on()')

    async def rdyp_off(self) -> None:
        """Deactivate RDYP to cut power to the robot's hardware."""
        await self.robot_brain.send('rdyp.off()')


class RobotSimulation(Robot):
    """A robot that consists of simulated modules.

    It regularly calls the step method of all modules to allow them to update their internal state.
    """

    def __init__(self, modules: list[Module]) -> None:
        super().__init__(modules)
        self._last_step: float | None = None
        rosys.on_repeat(self.step, 0.01, weak=True)

    async def step(self) -> None:
        now = rosys.time()
        if self._last_step is not None:
            dt = now - self._last_step
            for module in self.modules:
                await cast(ModuleSimulation, module).step(dt)
        self._last_step = now
