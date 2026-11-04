from .. import rosys
from ..helpers import remove_indentation
from .expander import ExpanderHardware
from .expander_bus import ExpanderBusHardware
from .module import ModuleHardware
from .robot_brain import RobotBrain

RELAY_PULSE = 0.5
"""Seconds the reset output stays on to release the battery relay."""


class BatteryControlHardware(ModuleHardware):

    def __init__(self, robot_brain: RobotBrain, *,
                 expander: ExpanderHardware | ExpanderBusHardware | None = None,
                 name: str = 'battery',
                 reset_pin: int = 15,
                 status_pin: int = 13,
                 ) -> None:
        """
        :param expander: the expander with the pins; an expander bus creates them in the expander's startup script,
            where the routine ``<name>_release`` times the relay pulse
        """
        self.name = name
        self.status: bool = False
        self.expander_bus = expander if isinstance(expander, ExpanderBusHardware) else None
        if self.expander_bus is not None:
            pulse_ms = round(RELAY_PULSE * 1000)
            lizard_code = self.expander_bus.add_lizard_code('\n'.join([
                f'{name}_reset = Output({reset_pin})',
                f'{name}_status = Input({status_pin})',
                f'int {name}_release_t = 0',
                f'let {name}_release do {name}_release_t = core.millis; {name}_reset.on(); '
                f'await core.millis > {name}_release_t + {pulse_ms}; {name}_reset.off(); end',
            ]), [(f'{name}_status.level', 'int')])
            core_message_fields = [f'{self.expander_bus.name}.{name}_status.level']
        else:
            lizard_code = remove_indentation(f'''
                {self.name}_reset = {expander.name + "." if expander else ""}Output({reset_pin})
                {self.name}_status = {expander.name + "." if expander else ""}Input({status_pin})
            ''')
            core_message_fields = [f'{self.name}_status.level']
        super().__init__(robot_brain=robot_brain, lizard_code=lizard_code, core_message_fields=core_message_fields)

    def handle_core_output(self, _: float, words: list[str]) -> None:
        self.status = int(words.pop(0)) == 1

    async def release_battery_relay(self) -> None:
        self.log.info('releasing battery relay')
        await rosys.sleep(1)
        if self.expander_bus is not None:
            # NOTE: the expander times the pulse, so that a lost second command cannot leave the relay on
            await self.expander_bus.send_after_startup(f'{self.name}_release()')
            await rosys.sleep(RELAY_PULSE)
            return
        await self.relay_on()
        await rosys.sleep(RELAY_PULSE)
        await self.relay_off()

    async def relay_on(self) -> None:
        """Switch on the relay that is used to cut charging power to the battery."""
        await self._send(f'{self.name}_reset.on()')

    async def relay_off(self) -> None:
        """Switch off the relay that is used to cut charging power to the battery."""
        await self._send(f'{self.name}_reset.off()')

    async def _send(self, command: str) -> None:
        if self.expander_bus is not None:
            await self.expander_bus.send_after_startup(command)
        else:
            await self.robot_brain.send(command)
