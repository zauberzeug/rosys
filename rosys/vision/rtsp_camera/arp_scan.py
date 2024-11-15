import os
import sys
from collections.abc import AsyncIterator

import ifaddr

from ... import rosys
from .vendors import VendorType, mac_to_vendor


def get_network_interface() -> str | None:
    """Return the first network interface that is not a loopback or virtual interface."""
    for interface in ifaddr.get_adapters():
        if interface.name.startswith(('lo', 'can', 'docker', 'veth', 'br')):
            continue
        if interface.ips is None:
            continue
        return interface.name
    return None


async def run_arp_scan(interface: str | None = None) -> str:
    """Run an arp-scan on the given interface and return the output.
    :param interface: The network interface to use for the arp-scan.
    """
    if sys.platform.startswith('darwin'):
        arpscan_cmd = 'arp-scan'
    else:
        arpscan_cmd = '/usr/sbin/arp-scan'  # TODO is this necessary? sbin should be in path
    if os.getuid() != 0:
        arpscan_cmd = f'sudo {arpscan_cmd}'

    cmd = f'{arpscan_cmd} --localnet'
    if not interface:
        interface = get_network_interface()
    if interface:
        cmd += f' --interface {interface}'

    output = await rosys.run.sh(cmd, timeout=10)

    if 'sudo' in output and 'name resolution' not in output:
        raise RuntimeError('Could not run arp-scan! Make sure it is installed and can be run with sudo.'
                           'Try running sudo visudo and add the following line: "rosys ALL=(ALL) NOPASSWD: /usr/sbin/arp-scan"')
    if 'Could not obtain MAC address for interface' in output:
        raise RuntimeError(f'arp-scan failed to obtain MAC address for interface {interface}')
    return output


async def find_cameras(network_interface: str | None = None) -> AsyncIterator[tuple[str, str]]:
    """Find all cameras in the local network and return MAC and IP addresses.
    :param network_interface: The network interface to use for the arp-scan.
    """
    output = await run_arp_scan(interface=network_interface)
    if 'ERROR' in output:
        return
    for line in output.splitlines():
        infos = line.split()
        if len(infos) < 2:
            continue
        mac = infos[1]
        ip = infos[0]
        if mac in ['packets', 'arp-scan'] or ip == 'Interface:':
            continue
        yield mac, ip


async def find_known_cameras(network_interface: str | None = None) -> list[tuple[str, str]]:
    """Find all cameras of known vendors and return MAC addresses.
    :param network_interface: The network interface to use for the arp-scan.
    """
    return [(mac, ip) async for mac, ip in find_cameras(network_interface=network_interface) if mac_to_vendor(mac) != VendorType.OTHER]


async def find_ip(mac: str, network_interface: str | None = None) -> str | None:
    """Find the IP address of a camera with the given MAC address."""
    async for mac_, ip in find_cameras(network_interface=network_interface):
        if mac_ == mac:
            return ip
    return None
