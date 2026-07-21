import logging
import re
import shutil
from dataclasses import dataclass

from ... import rosys


@dataclass(slots=True, kw_only=True)
class NetworkStats:
    tx_errors: int
    tx_dropped: int
    rx_errors: int
    rx_dropped: int

    def __gt__(self, other) -> bool:
        return self.tx_errors > other.tx_errors or \
            self.tx_dropped > other.tx_dropped or \
            self.rx_errors > other.rx_errors or \
            self.rx_dropped > other.rx_dropped

    def msg(self) -> str:
        return f'{self.tx_errors}/{self.rx_errors} errors and {self.tx_dropped}/{self.rx_dropped} dropped'


class NetworkMonitor:

    def __init__(self) -> None:
        self.log = logging.getLogger('rosys.network_monitor')
        self.interfaces: dict[str, NetworkStats] = {}

        rosys.on_repeat(self.step, 60)

    @staticmethod
    def is_operable() -> bool:
        return shutil.which('ip') is not None

    async def step(self) -> None:
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
            elif stats > self.interfaces[name]:
                msg = name + ' has ' + stats.msg()
                self.log.warning(msg)
                rosys.notify(msg, 'warning')
                self.interfaces[name] = stats

    @staticmethod
    def split_interfaces(output: str) -> list[str]:
        spacing = re.sub('^[0-9]+: ', '\n', output, flags=re.MULTILINE)
        return spacing.strip().split('\n\n')
