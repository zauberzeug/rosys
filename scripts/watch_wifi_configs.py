#!/usr/bin/env python3

import os
import subprocess
import time
from watchdog.observers import Observer
from watchdog.events import FileSystemEvent, FileSystemEventHandler


def nmcli(cmd: str) -> None:
    cmd = 'sudo nmcli connection' + cmd
    subprocess.Popen(cmd, shell=True)


def apply_wifi_configurations(dir: str, interface: str = 'wlan0') -> None:
    for network in os.scandir(dir):
        with open(network.path) as f:
            password = f.read()

        print(f'creating new network {ssid}')
        ssid = network.name
        nmcli(f'down "{ssid}"')
        nmcli(f'del "{ssid}"')
        if password:
            password = f'wifi-sec.key-mgmt wpa-psk wifi-sec.psk "{password}"'
        nmcli(f'add type wifi ifname {interface} con-name "{ssid}" ssid "{ssid}" {password}')
        nmcli(f'modify "{ssid}" connection.interface-name {interface} ipv6.method "ignore" wifi.mac-address ""')


class WifiChangedEventHandler(FileSystemEventHandler):
    def on_any_event(self, event: FileSystemEvent):
        apply_wifi_configurations(os.path.dirname(event.src_path))


if __name__ == '__main__':

    observer = Observer()
    observer.schedule(WifiChangedEventHandler(), '/home/zauberzeug/.rosys/wifi', recursive=False)
    observer.start()
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        observer.stop()
    observer.join()
