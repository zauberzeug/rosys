from nicegui.ui import Ui
import logging
import socket
import os.path
from pathlib import Path
import asyncio

log = logging.getLogger('rosys.wifi')


def has_internet() -> bool:
    '''Returns True if there's a connection'''

    IP_ADDRESS_LIST = [
        '1.1.1.1',  # Cloudflare
        '1.0.0.1',
        '8.8.8.8',  # Google DNS
        '8.8.4.4',
        '208.67.222.222',  # Open DNS
        '208.67.220.220',
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
        log.exception('No internet connection')
        return False


def create_wifi(ui: Ui):

    def add(ssid: str, password: str):
        log.info(f'adding {ssid}, {password}')
        wifi_configs = os.path.expanduser(f'~/.rosys/wifi')
        Path(wifi_configs).mkdir(parents=True, exist_ok=True)
        # NOTE a daemon on the host system must watch the .rosys/wifi dir and reload the configuration with nmcli or similar
        with open(f'{wifi_configs}/{ssid}', 'w') as f:
            f.write(password)
        dialog.close()

    async def update_wifi_status():
        if await asyncio.get_event_loop().run_in_executor(None, has_internet):
            wifi_button.props(replace='color=green')
            status.set_text(f'Robot is connected to the internet.')
        else:
            wifi_button.props(replace='color=red')
            status.set_text(f'Robot is not connected to the internet.')
        return False  # do not refresh UI

    with ui.dialog() as dialog, ui.card():
        status = ui.label()
        ssid = ui.input('ssid')
        password = ui.input('password')
        with ui.row():
            ui.button('Add', on_click=lambda: add(ssid.value, password.value))
            ui.button('Close', on_click=dialog.close)

    wifi_button = ui.button(on_click=dialog.open).props('icon=network_wifi')
    ui.timer(5, callback=update_wifi_status)
