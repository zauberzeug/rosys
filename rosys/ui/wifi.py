from nicegui.ui import Ui
import logging
import socket
import os
from os.path import expanduser
import subprocess
import platform
from pathlib import Path

log = logging.getLogger('rosys.wifi')


def check_internet_connection():
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
    if platform.system() == 'Linux':
        subprocess.Popen(cmd, shell=True)
    else:
        log.info(cmd)


def apply_wifi_configurations(dir: str, interface: str = 'wlan0') -> None:
    for network in os.scandir(dir):
        ssid = network.name
        nmcli(f'down "{ssid}"')
        nmcli(f'del "{ssid}"')
        with open(network.path) as f:
            password = f.read()
            if password:
                password = f'wifi-sec.key-mgmt wpa-psk wifi-sec.psk "{password}"'
        nmcli(f'add type wifi ifname {interface} con-name "{ssid}" ssid "{ssid}" {password}')
        nmcli(f'modify "{ssid}" connection.interface-name {interface} ipv6.method "ignore" wifi.mac-address ""')


def create_wifi(ui: Ui):

    def show_wifi():
        connected = check_internet_connection()
        message = f'Robot is {"" if connected else "not "}connected to the internet.'
        status.set_text(message)
        log.info(message)
        dialog.open()

    def add(ssid: str, password: str):
        log.info(f'adding {ssid}, {password}')
        wifi_configs = expanduser(f'~/.rosys/wifi')
        Path(wifi_configs).mkdir(parents=True, exist_ok=True)

        with open(f'{wifi_configs}/{ssid}', 'w') as f:
            f.write(password)
        dialog.close()
        apply_wifi_configurations(wifi_configs)

    with ui.dialog() as dialog:
        with ui.card():
            status = ui.label('status label')
            ssid = ui.input('ssid')
            password = ui.input('password')
            with ui.row():
                ui.button('Add', on_click=lambda: add(ssid.value, password.value))
                ui.button('Close', on_click=dialog.close)

    ui.button(on_click=show_wifi).props('icon=network_wifi')
