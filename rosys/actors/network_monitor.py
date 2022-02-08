from dataclasses import dataclass
import re
import shutil
import rosys
from .actor import Actor


@dataclass
class NetworkStats:
    tx_errors: int
    tx_dropped: int
    rx_errors: int
    rx_dropped: int

    def __gt__(self, other):
        return self.tx_errors > other.tx_errors or \
            self.tx_dropped > other.tx_dropped or \
            self.rx_errors > other.rx_errors or \
            self.rx_dropped > other.rx_dropped

    def msg(self):
        return f'{self.tx_errors}/{self.rx_errors} errors and {self.tx_dropped}/{self.rx_dropped} dropped'


class NetworkMonitor(Actor):
    interval: float = 60

    def __init__(self) -> None:
        super().__init__()
        self.interfaces: dict[str, NetworkStats] = {}

    @staticmethod
    def is_operable() -> bool:
        return shutil.which('ip') is not None

    async def step(self):
        await super().step()
        output = await rosys.run.sh(['ip', '-s', 'a'])
        for interface in NetworkMonitor.split_interfaces(output):
            name = interface[:interface.index(':')]
            lines = interface.split('\n')
            stats = NetworkStats(
                tx_errors=int(lines[-1].split()[2]),
                tx_dropped=int(lines[-1].split()[3]),
                rx_errors=int(lines[-3].split()[2]),
                rx_dropped=int(lines[-3].split()[3]),
            )
            if name not in self.interfaces:
                self.interfaces[name] = stats
            else:
                if stats > self.interfaces[name]:
                    msg = name + ' has ' + stats.msg()
                    self.log.warning(msg)
                    await self.notify(msg)
                    self.interfaces[name] = stats

    @staticmethod
    def split_interfaces(output: str) -> list[str]:
        spacing = re.sub('^[0-9]+: ', '\n', output, flags=re.MULTILINE)
        return spacing.strip().split('\n\n')
