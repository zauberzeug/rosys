import re
import shutil
from .actor import Actor


class NetworkMonitor(Actor):
    interval: float = 6

    @staticmethod
    def is_operable() -> bool:
        return shutil.which('ip') is not None

    async def step(self):
        await super().step()
        output = await self.run_sh(['ip', '-s', 'a'])
        for interface in NetworkMonitor.split_interfaces(output):
            name = interface[:interface.index(':')]
            lines = interface.split('\n')
            tx_errors = int(lines[-1].split()[2])
            tx_dropped = int(lines[-1].split()[3])
            rx_errors = int(lines[-1].split()[2])
            rx_dropped = int(lines[-3].split()[3])
            if any([v > 0 for v in [tx_errors, tx_dropped, rx_errors, rx_dropped]]):
                msg = f'{name} has {tx_errors} tx errors, {tx_dropped} tx_dropped, {rx_errors} rx errors and {rx_dropped} rx_dropped'
                self.log.warning(msg)
                await self.notify(msg)

    @ staticmethod
    def split_interfaces(output: str) -> list[str]:
        spacing = re.sub('^[0-9]+: ', '\n', output, flags=re.MULTILINE)
        return spacing.strip().split('\n\n')
