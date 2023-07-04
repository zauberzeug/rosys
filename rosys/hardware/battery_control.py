from typing import Optional

import rosys

from .expander import ExpanderHardware
from .module import ModuleHardware
from .robot_brain import RobotBrain


class BatteryControlHardware(ModuleHardware):

    def __init__(self, robot_brain: RobotBrain, *,
                 expander: Optional[ExpanderHardware] = None,
                 name: str = 'battery',
                 reset_pin: int = 15,
                 status_pin: int = 13,
                 ) -> None:
        self.name = name
        lizard_code = f'{self.name}_reset = {expander.name + "." if expander else ""}Output({reset_pin})\n'
        lizard_code += f'{self.name}_status = {expander.name + "." if expander else ""}Input({status_pin})\n'
        super().__init__(robot_brain=robot_brain, lizard_code=lizard_code)

    async def release_battery_relais(self) -> None:
        self.log.info('releasing battery relais')
        await rosys.sleep(1)
        await self.robot_brain.send('battery_reset.on()')
        await rosys.sleep(0.5)
        await self.robot_brain.send('battery_reset.off()')
