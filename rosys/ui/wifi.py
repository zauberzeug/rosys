from nicegui.ui import Ui
import logging
import socket
import os
from os.path import expanduser
import subprocess
import platform
from pathlib import Path
from random import randrange

log = logging.getLogger('rosys.wifi')


def has_internet():
    """ Returns True if there's a connection """

    IP_ADDRESS_LIST = [
        "1.1.1.1",  # Cloudflare
        "1.0.0.1",
        "8.8.8.8",  # Google DNS
        "8.8.4.4",
        "208.67.222.222",  # Open DNS
        "208.67.220.220"
    ]

    port = 53
    timeout = 3

    for host in IP_ADDRESS_LIST:
        try:
            socket.setdefaulttimeout(timeout)
            socket.socket(socket.AF_INET, socket.SOCK_STREAM).connect((host, port))
            return True
        except socket.error:
            pass
    else:
        log.exception("No internet connection")
        return False


def nmcli(cmd: str) -> None:
    cmd = 'sudo nmcli connection' + cmd
    subprocess.Popen(cmd, shell=True)


def apply_wifi_configurations(dir: str, interface: str = 'wlan0') -> None:
    for network in os.scandir(dir):
        with open(network.path) as f:
            password = f.read()

        if platform.system() != 'Linux':
            log.info(f'configured network: {network.name}:{password}')
            continue

        ssid = network.name
        nmcli(f'down "{ssid}"')
        nmcli(f'del "{ssid}"')
        if password:
            password = f'wifi-sec.key-mgmt wpa-psk wifi-sec.psk "{password}"'
        nmcli(f'add type wifi ifname {interface} con-name "{ssid}" ssid "{ssid}" {password}')
        nmcli(f'modify "{ssid}" connection.interface-name {interface} ipv6.method "ignore" wifi.mac-address ""')


def create_wifi(ui: Ui):

    def add(ssid: str, password: str):
        log.info(f'adding {ssid}, {password}')
        wifi_configs = expanduser(f'~/.rosys/wifi')
        Path(wifi_configs).mkdir(parents=True, exist_ok=True)

        with open(f'{wifi_configs}/{ssid}', 'w') as f:
            f.write(password)
        dialog.close()
        apply_wifi_configurations(wifi_configs)

    def update_wifi_status():
        if has_internet():
            wifi_button.style('color:green', remove='color')
            status.set_text(f'Robot is connected to the internet.')
        else:
            wifi_button.style('color:red', remove='color')
            status.set_text(f'Robot is not connected to the internet.')
        return False  # do not refresh UI

    with ui.dialog() as dialog:
        with ui.card():
            status = ui.label('status label')
            ssid = ui.input('ssid')
            password = ui.input('password')
            with ui.row():
                ui.button('Add', on_click=lambda: add(ssid.value, password.value))
                ui.button('Close', on_click=dialog.close)

    wifi_button = ui.button(on_click=dialog.open).props('icon=network_wifi')
    ui.timer(5, callback=update_wifi_status)
