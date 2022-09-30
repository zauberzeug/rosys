import asyncio
import logging
import os
import socket

from nicegui import ui

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
        log.warning('No internet connection')
        return False


class WifiButton(ui.button):
    '''The WiFi button indicates the current connectivity state and allows setting a new WiFi connection.'''

    def __init__(self) -> None:
        with ui.dialog() as self.dialog, ui.card():
            self.status = ui.label()
            self.ssid = ui.input('ssid')
            self.password = ui.input('password')
            with ui.row():
                ui.button('Add', on_click=self.add)
                ui.button('Close', on_click=self.dialog.close)

        super().__init__(on_click=self.dialog.open)
        self.props('icon=network_wifi')
        ui.timer(5, callback=self.update_wifi_status)

    def add(self) -> None:
        log.info(f'adding {self.ssid.value}, {self.password.value}')
        wifi_configs = os.path.expanduser(f'~/.rosys/wifi')
        os.makedirs(wifi_configs, exist_ok=True)
        # NOTE a daemon on the host system must watch the .rosys/wifi dir and reload the configuration with nmcli or similar
        with open(f'{wifi_configs}/{self.ssid.value}', 'w') as f:
            f.write(self.password.value)
        self.dialog.close()

    async def update_wifi_status(self) -> None:
        if await asyncio.get_event_loop().run_in_executor(None, has_internet):
            self.props('color=green')
            self.status.set_text(f'Robot is connected to the internet.')
        else:
            self.props('color=red')
            self.status.set_text(f'Robot is not connected to the internet.')
