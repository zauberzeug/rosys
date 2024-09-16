import logging
from collections import deque

from nicegui import ui

from .. import rosys
from ..event import Event
from .communication import Communication
from .lizard_firmware import LizardFirmware

CLOCK_OFFSET_HISTORY_LENGTH = 100


class RobotBrain:
    """This module manages the communication with a [Zauberzeug Robot Brain](https://zauberzeug.com/products/robot-brain).

    It expects a communication object, which is used for the actual read and write operations.
    Besides providing some basic methods like configuring or restarting the microcontroller, it augments and verifies checksums for each message.

    It also keeps track of the clock offset between the microcontroller and the host system, which is used to synchronize the hardware time with the system time.
    The clock offset is calculated by comparing the hardware time with the system time and averaging the differences over a number of samples.
    If the offset changes significantly, a notification is sent and the offset history is cleared.
    """

    def __init__(self, communication: Communication, *, enable_esp_on_startup: bool = True) -> None:
        self.LINE_RECEIVED = Event()
        """a line has been received from the microcontroller (argument: line as string)"""
        self.FLASH_P0_COMPLETE = Event()
        """flashing p0 was successful and 'Replica complete' was received"""

        self.log = logging.getLogger('rosys.robot_rain')

        self.communication = communication
        self.lizard_code = ''
        self.lizard_firmware = LizardFirmware(self)

        self.waiting_list: dict[str, str | None] = {}
        self._clock_offset: float | None = None
        self._clock_offsets: deque[float] = deque(maxlen=CLOCK_OFFSET_HISTORY_LENGTH)
        self.hardware_time: float | None = None
        if enable_esp_on_startup:
            rosys.on_startup(self.enable_esp)

    @property
    def clock_offset(self) -> float | None:
        return self._clock_offset

    def developer_ui(self) -> None:
        version_select: ui.select

        async def read_online_versions() -> None:
            await self.lizard_firmware.read_online_version()
            version_select.set_options(list(self.lizard_firmware.online_versions.keys()))

        async def online_update() -> None:
            await self.lizard_firmware.download()
            await self.lizard_firmware.flash_core()
            await self.restart()
            await self.configure()
            await self.lizard_firmware.flash_p0()

        async def local_update() -> None:
            await self.lizard_firmware.flash_core()
            await self.restart()
            await self.configure()
            await self.lizard_firmware.flash_p0()

        with ui.row().classes('items-center'):
            version_select = ui.select([], label='Lizard Version').style('min-width: 140px;') \
                .bind_value(self.lizard_firmware, 'selected_online_version')
            ui.button(on_click=read_online_versions).props('icon=refresh flat round dense') \
                .tooltip('Read all available versions from GitHub')
            online_update_button = ui.button(on_click=online_update).props('icon=file_download flat round dense') \
                .tooltip('Download and flash online version to Core and P0 microcontrollers')
        with ui.row().classes('items-center'):
            ui.label().bind_text_from(self.lizard_firmware, 'local_version', backward=lambda x: f'Local: {x or "?"}')
            local_update_button = ui.button(on_click=local_update).props('icon=file_download flat round dense') \
                .tooltip('Flash local version to Core and P0 microcontrollers')
        with ui.row().classes('items-center'):
            ui.label().bind_text_from(self.lizard_firmware, 'core_version', backward=lambda x: f'Core: {x or "?"}')
            configure_button = ui.button(on_click=self.configure).props('icon=build flat round dense') \
                .tooltip('Configure microcontrollers')
        with ui.row().classes('items-center'):
            ui.label().bind_text_from(self.lizard_firmware, 'p0_version', backward=lambda x: f'P0: {x or "?"}')

        def update_visibility() -> None:
            online_update_button.visible = \
                self.lizard_firmware.selected_online_version != self.lizard_firmware.core_version or \
                self.lizard_firmware.selected_online_version != self.lizard_firmware.p0_version
            local_update_button.visible = \
                self.lizard_firmware.local_version != self.lizard_firmware.core_version or \
                self.lizard_firmware.local_version != self.lizard_firmware.p0_version
            configure_button.visible = \
                self.lizard_firmware.local_checksum != self.lizard_firmware.core_checksum
        ui.timer(1.0, update_visibility)

        with ui.row().classes('items-center'):
            ui.button('Refresh', on_click=self.lizard_firmware.read_all).props('outline') \
                .tooltip('Read installed and available versions')
            with ui.row():
                with ui.menu() as menu:
                    ui.menu_item('Download', on_click=self.lizard_firmware.download) \
                        .tooltip('Download the latest Lizard firmware from GitHub')
                    ui.menu_item('Flash Core', on_click=self.lizard_firmware.flash_core) \
                        .tooltip('Flash the downloaded Lizard firmware to the Core microcontroller')
                    ui.menu_item('Flash P0', on_click=self.lizard_firmware.flash_p0) \
                        .tooltip('Flash the downloaded Lizard firmware to the P0 microcontroller')
                    ui.menu_item('Enable', on_click=self.enable_esp) \
                        .tooltip('Enable the microcontroller module (will later be done automatically)')
                    ui.menu_item('Configure', on_click=self.configure) \
                        .tooltip('Configure the microcontroller with the Lizard startup file')
                    ui.menu_item('Restart', on_click=self.restart) \
                        .tooltip('Restart the microcontroller')
                ui.button(on_click=menu.open).props('icon=more_vert flat round')

        ui.label().bind_text_from(self, 'clock_offset', lambda offset: f'Clock offset: {offset or 0:.3f} s')

    async def configure(self) -> None:
        rosys.notify('Configuring Lizard...')
        await self.send('!-')
        for line in self.lizard_code.splitlines():
            await self.send(f'!+{line}')
        await self.send('!.')
        await self.restart()
        rosys.notify('Lizard configured successfully.', 'positive')

    async def restart(self) -> None:
        await self.send('core.restart()')

    async def read_lines(self) -> list[tuple[float, str]]:
        lines: list[tuple[float, str]] = []
        millis = None
        while True:
            unchecked = await self.communication.read()
            line = check(unchecked)
            if not line:
                break
            words = line.split()
            if not words:
                continue
            first = words.pop(0)
            if first in self.waiting_list:
                self.waiting_list[first] = line
            if first == 'core':
                millis = float(words.pop(0))
                if self.clock_offset is None:
                    continue
                self.hardware_time = millis / 1000 + self.clock_offset
            if 'Replica complete' in line:
                self.FLASH_P0_COMPLETE.emit()
            self.LINE_RECEIVED.emit(line)
            if self.hardware_time is None:
                continue
            lines.append((self.hardware_time, line))
        if millis is not None:
            self._handle_clock_offset(rosys.time() - millis / 1000)
        return lines

    def _handle_clock_offset(self, offset: float) -> None:
        if self._clock_offset is not None and abs(offset - self._clock_offset) > 0.1:
            self.log.info('Clock offset changed from %.3f to %.3f', self._clock_offset, offset)
            self._clock_offsets.clear()
        self._clock_offsets.append(offset)
        self._clock_offset = sum(self._clock_offsets) / len(self._clock_offsets)

    async def send(self, msg: str) -> None:
        await self.communication.send(augment(msg))

    async def send_and_await(self, msg: str, ack: str, *, timeout: float = float('inf')) -> str | None:
        self.waiting_list[ack] = None
        await self.send(msg)
        t0 = rosys.time()
        while self.waiting_list.get(ack) is None and rosys.time() < t0 + timeout:
            await rosys.sleep(0.1)
        return self.waiting_list.pop(ack) if ack in self.waiting_list else None

    async def enable_esp(self) -> None:
        rosys.notify('Enabling ESP...')
        command = ['sudo', './flash.py', *self.lizard_firmware.flash_params, 'enable']
        output = await rosys.run.sh(command, timeout=None, working_dir=self.lizard_firmware.PATH)
        self.log.debug(output)
        rosys.notify('Enabling ESP: done', 'positive')

    def __del__(self) -> None:
        self.communication.disconnect()

    def __repr__(self) -> str:
        return f'<RobotBrain {self.communication}>'


def augment(line: str) -> str:
    checksum = 0
    for c in line:
        checksum ^= ord(c)
    return f'{line}@{checksum:02x}'


def check(line: str | None) -> str:
    if line is None:
        return ''
    if line[-3:-2] != '@':
        return ''
    check_ = int(line[-2:], 16)
    line = line[:-3]
    checksum = 0
    for c in line:
        checksum ^= ord(c)
    if checksum != check_:
        return ''
    return line
