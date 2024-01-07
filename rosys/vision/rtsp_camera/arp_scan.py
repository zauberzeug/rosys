import os
import sys
from typing import AsyncIterator, Optional

import netifaces

from ... import rosys
from .vendors import VendorType, mac_to_vendor


async def run_arp_scan(interface: str) -> str:
    """Run an arp-scan on the given interface and return the output."""
    if sys.platform.startswith('darwin'):
        arpscan_cmd = 'arp-scan'
    else:
        arpscan_cmd = '/usr/sbin/arp-scan'  # TODO is this necessary? sbin should be in path
    if os.getuid() != 0:
        arpscan_cmd = f'sudo {arpscan_cmd}'
    cmd = f'{arpscan_cmd} -I {interface} --localnet'
    output = await rosys.run.sh(cmd, timeout=10)
    if 'sudo' in output:
        raise RuntimeError('Could not run arp-scan! Make sure it is installed and can be run with sudo.'
                           'Try running sudo visudo and add the following line: "rosys ALL=(ALL) NOPASSWD: /usr/sbin/arp-scan"')
    return output


async def find_cameras() -> AsyncIterator[tuple[str, str]]:
    """Find all cameras in the local network and return MAC and IP addresses."""
    for interface in netifaces.interfaces():
        output = await run_arp_scan(interface)
        if 'ERROR' in output:
            continue
        for line in output.splitlines():
            infos = line.split()
            if len(infos) < 2:
                continue
            mac = infos[1]
            ip = infos[0]
            yield mac, ip


async def find_known_cameras() -> list[str]:
    """Find all cameras of known vendors and return MAC addresses."""
    return [mac async for mac, _ in find_cameras() if mac_to_vendor(mac) != VendorType.OTHER]


async def find_ip(mac: str) -> Optional[str]:
    async for mac_, ip in find_cameras():
        if mac_ == mac:
            return ip
    return None
