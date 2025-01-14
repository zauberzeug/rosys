from .. import rosys
from ..helpers import remove_indentation
from .expander import ExpanderHardware
from .module import ModuleHardware
from .robot_brain import RobotBrain


class BatteryControlHardware(ModuleHardware):

    def __init__(self, robot_brain: RobotBrain, *,
                 expander: ExpanderHardware | None = None,
                 name: str = 'battery',
                 reset_pin: int = 15,
                 status_pin: int = 13,
                 ) -> None:
        self.name = name
        self.status: bool = False
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
        await self.robot_brain.send(f'{self.name}_reset.on()')
        await rosys.sleep(0.5)
        await self.robot_brain.send(f'{self.name}_reset.off()')
