#!/usr/bin/env python3
import os
import subprocess
import time
from pathlib import Path

from watchdog.events import FileSystemEvent, FileSystemEventHandler
from watchdog.observers import Observer

PATH = Path('~/.rosys/wifi').expanduser()


def nmcli(cmd: str) -> None:
    cmd = f'nmcli connection {cmd}'
    if os.getuid() != 0:
        cmd = f'sudo {cmd}'
    subprocess.Popen(cmd, shell=True)


def apply_wifi_configurations(interface: str = 'wlan0') -> None:
    for filepath in PATH.glob('*'):
        ssid = filepath.name
        password = filepath.read_text()
        print(f'creating new network {ssid}')
        nmcli(f'down "{ssid}"')
        nmcli(f'del "{ssid}"')
        if password:
            password = f'wifi-sec.key-mgmt wpa-psk wifi-sec.psk "{password}"'
        nmcli(f'add type wifi ifname {interface} con-name "{ssid}" ssid "{ssid}" {password}')
        nmcli(f'modify "{ssid}" connection.interface-name {interface} ipv6.method "ignore" wifi.mac-address ""')


class WifiChangedEventHandler(FileSystemEventHandler):

    def on_any_event(self, _: FileSystemEvent) -> None:
        apply_wifi_configurations()


if __name__ == '__main__':
    apply_wifi_configurations()
    observer = Observer()
    observer.schedule(WifiChangedEventHandler(), PATH)
    observer.start()
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        observer.stop()
    observer.join()
