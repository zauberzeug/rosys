import asyncio
import logging
from collections import deque

from nicegui import ui

from .. import rosys
from ..event import Event
from .communication import Communication
from .esp_pins import EspPins
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

    def __init__(self, communication: Communication, *, enable_esp_on_startup: bool = True, use_espresso: bool = False) -> None:
        self.ESP_CONNECTED = Event[[]]()
        """ESP has been connected and Lizard is ready to use"""
        self.LINE_RECEIVED = Event[str]()
        """a line has been received from the microcontroller (argument: line as string)"""
        self.FLASH_P0_COMPLETE = Event[[]]()
        """flashing p0 was successful and 'Replica complete' was received"""

        self.log = logging.getLogger('rosys.robot_brain')

        self.communication = communication
        self.lizard_code = ''
        self.lizard_firmware = LizardFirmware(self)

        self.waiting_list: dict[str, str | None] = {}
        self._clock_offset: float | None = None
        self._clock_offsets: deque[float] = deque(maxlen=CLOCK_OFFSET_HISTORY_LENGTH)
        self._hardware_time: float | None = None
        self._use_espresso = use_espresso
        if enable_esp_on_startup:
            rosys.on_startup(self.enable_esp)

        self.esp_pins_core = EspPins(name='core', robot_brain=self)
        self.esp_pins_p0 = EspPins(name='p0', robot_brain=self)

        self._esp_lock = asyncio.Lock()

    @property
    def clock_offset(self) -> float | None:
        return self._clock_offset

    @property
    def hardware_time(self) -> float | None:
        return self._hardware_time

    @property
    def is_ready(self) -> bool:
        return self._hardware_time is not None

    def developer_ui(self) -> None:
        version_select: ui.select

        async def read_online_versions() -> None:
            await self.lizard_firmware.read_online_version()
            version_select.set_options(list(self.lizard_firmware.online_versions))
            if version_select.options:
                version_select.value = version_select.options[0]

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
                .bind_value_to(self.lizard_firmware, 'selected_online_version')
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
                    ui.menu_item('Download Lizard', on_click=self.lizard_firmware.download) \
                        .tooltip('Download the latest Lizard firmware from GitHub')

                    async def check_strapping_pins_core():
                        ui.notify(await self.esp_pins_core.get_strapping_pins())
                    ui.menu_item('Check Core Strapping Pins', on_click=check_strapping_pins_core) \
                        .tooltip('Check if the Core strapping pins are in the correct state for flashing.')
                    ui.menu_item('Flash Core', on_click=self.lizard_firmware.flash_core) \
                        .tooltip('Flash the downloaded Lizard firmware to the Core microcontroller')

                    async def check_strapping_pins_p0():
                        ui.notify(await self.esp_pins_p0.get_strapping_pins())
                    ui.menu_item('Check P0 Strapping Pins', on_click=check_strapping_pins_p0) \
                        .tooltip('Check if the P0 strapping pins are in the correct state for flashing.')
                    ui.menu_item('Flash P0', on_click=self.lizard_firmware.flash_p0) \
                        .tooltip('Flash the downloaded Lizard firmware to the P0 microcontroller')

                    ui.separator()
                    ui.menu_item('Configure', on_click=self.configure) \
                        .tooltip('Configure the microcontroller with the Lizard startup file')
                    ui.menu_item('Download Config', on_click=lambda: ui.download(self.lizard_code.encode('utf-8'), 'config.liz')) \
                        .tooltip('Download the Lizard config file')

                    ui.separator()
                    ui.menu_item('Enable', on_click=self.enable_esp) \
                        .tooltip('Enable the microcontroller')
                    ui.menu_item('Disable', on_click=self.disable_esp) \
                        .tooltip('Disable the microcontroller')
                    ui.menu_item('Reset', on_click=self.reset_esp) \
                        .tooltip('Reset the microcontroller')
                    ui.menu_item('Restart', on_click=self.restart) \
                        .tooltip('Restart the microcontroller')
                ui.button(on_click=menu.open).props('icon=more_vert flat round')

        ui.label().bind_text_from(self, 'clock_offset', lambda offset: f'Clock offset: {offset or 0:.3f} s')
        ui.label().bind_text_from(self, 'is_ready', lambda ready: f'Ready: {ready}')

    async def configure(self) -> None:
        rosys.notify('Configuring Lizard...')
        await self.send('!-', force=True)
        for line in self.lizard_code.splitlines():
            await self.send(f'!+{line}', force=True)
        await self.send('!.', force=True)
        await self.restart()
        rosys.notify('Lizard configured successfully.', 'positive')

    async def restart(self) -> None:
        await self.send('core.restart()', force=True)
        try:
            await self.LINE_RECEIVED.emitted(timeout=1.0)  # NOTE: we have to wait for the last core message to be sent
        except TimeoutError:
            pass
        finally:
            self._hardware_time = None

    async def read_lines(self) -> list[tuple[float, str]]:
        lines: list[tuple[float, str]] = []
        millis = None
        while True:
            unchecked = await self.communication.read()
            if self._esp_lock.locked():
                break
            line = check(unchecked)
            if not line:
                break
            words = line.split()
            if not words:
                continue
            first = words.pop(0)
            if first in self.waiting_list:
                self.waiting_list[first] = line
            hardware_time: float | None = None
            if first == 'core':
                millis = float(words.pop(0))
                if self.clock_offset is None:
                    continue
                hardware_time = millis / 1000 + self.clock_offset
            if 'Replica complete' in line:
                self.FLASH_P0_COMPLETE.emit()
            self.LINE_RECEIVED.emit(line)
            if hardware_time is not None:
                if self._hardware_time is None:
                    rosys.notify('ESP connected', 'positive')
                    self.ESP_CONNECTED.emit()
                self._hardware_time = hardware_time
            if self._hardware_time is None:
                continue
            lines.append((self._hardware_time, line))
        if millis is not None:
            self._handle_clock_offset(rosys.time() - millis / 1000)
        return lines

    def _handle_clock_offset(self, offset: float) -> None:
        if self._clock_offset is not None and abs(offset - self._clock_offset) > 0.1:
            self.log.info('Clock offset changed from %.3f to %.3f', self._clock_offset, offset)
            self._clock_offsets.clear()
        self._clock_offsets.append(offset)
        self._clock_offset = sum(self._clock_offsets) / len(self._clock_offsets)

    async def send(self, msg: str, *, force: bool = False) -> None:
        """Send a Lizard command to the ESP.

        :param msg: The Lizard command to send
        :param force: Whether to send the message even if the ESP is not ready
        :raises EspNotReadyException: When the ESP is not ready and force is ``False``
        """
        if not self.is_ready and not force:
            raise EspNotReadyException('Sending message failed because ESP is not ready')
        await self.communication.send(augment(msg))

    async def send_and_await(self, msg: str, ack: str, *, timeout: float = float('inf'), force: bool = False) -> str | None:
        """Send a Lizard command to the ESP and await a response.

        :param msg: The Lizard command to send
        :param ack: The first word of the response message to wait for
        :param timeout: Response timeout
        :param force: Whether to send the message even if the ESP is not ready
        :raises EspNotReadyException: When the ESP is not ready and force is ``False``
        :return: The response message or ``None`` if the timeout is reached
        """
        if not self.is_ready and not force:
            raise EspNotReadyException('Sending message failed because ESP is not ready')
        self.waiting_list[ack] = None
        await self.send(msg, force=force)
        t0 = rosys.time()
        while self.waiting_list.get(ack) is None and rosys.time() < t0 + timeout:
            await rosys.sleep(0.1)
        return self.waiting_list.pop(ack) if ack in self.waiting_list else None

    async def enable_esp(self) -> None:
        if self._esp_lock.locked():
            return
        if self._use_espresso:
            await self._enable_espresso()
            return
        self.log.warning("Lizard's flash.py will be deprecated in the future. "
                         'Consider updating Lizard and using the new espresso.py instead.')
        async with self._esp_lock:
            self._hardware_time = None
            rosys.notify('Enabling ESP...')
            command = ['sudo', './flash.py', *self.lizard_firmware.flash_params, 'enable']
            output = await rosys.run.sh(command, timeout=None, working_dir=self.lizard_firmware.PATH)
            self.log.debug(output)
            rosys.notify('Enabling ESP: done')

    async def _enable_espresso(self) -> None:
        if self._esp_lock.locked():
            return
        async with self._esp_lock:
            self._hardware_time = None
            rosys.notify('Enabling ESP...')
            command = ['sudo', './espresso.py', 'enable',
                       *self._convert_flash_params(self.lizard_firmware.flash_params)]
            self.log.debug('enable: %s', command)
            output = await rosys.run.sh(command, timeout=None, working_dir=self.lizard_firmware.PATH)
            if 'Finished.' in output:
                self.log.debug(output)
                rosys.notify('Enabling ESP: done', 'positive')
            else:
                self.log.error(output)
                rosys.notify('Enabling ESP: failed', 'negative')

    async def disable_esp(self) -> None:
        if self._esp_lock.locked():
            return
        async with self._esp_lock:
            self._hardware_time = None
            rosys.notify('Disabling ESP...')
            command = ['sudo', './espresso.py', 'disable', *
                       self._convert_flash_params(self.lizard_firmware.flash_params)]
            self.log.debug('disable: %s', command)
            output = await rosys.run.sh(command, timeout=None, working_dir=self.lizard_firmware.PATH)
            if 'Finished.' in output:
                self.log.debug(output)
                rosys.notify('Disabling ESP: done', 'positive')
            else:
                self.log.error(output)
                rosys.notify('Disabling ESP: failed', 'negative')

    async def reset_esp(self) -> None:
        if self._esp_lock.locked():
            return
        if self._use_espresso:
            await self._reset_espresso()
            return
        self.log.warning("Lizard's flash.py will be deprecated in the future. "
                         'Consider updating Lizard and using the new espresso.py instead.')
        async with self._esp_lock:
            self._hardware_time = None
            rosys.notify('Resetting ESP...')
            command = ['sudo', './flash.py', *self.lizard_firmware.flash_params, 'reset']
            output = await rosys.run.sh(command, timeout=None, working_dir=self.lizard_firmware.PATH)
            self.log.debug(output)
            rosys.notify('Resetting ESP: done')

    async def _reset_espresso(self) -> None:
        if self._esp_lock.locked():
            return
        async with self._esp_lock:
            self._hardware_time = None
            rosys.notify('Resetting ESP...')
            command = ['sudo', './espresso.py', 'reset', *self._convert_flash_params(self.lizard_firmware.flash_params)]
            self.log.debug('reset: %s', command)
            output = await rosys.run.sh(command, timeout=None, working_dir=self.lizard_firmware.PATH)
            if 'Finished.' in output:
                self.log.debug(output)
                rosys.notify('Resetting ESP: done', 'positive')
            else:
                self.log.error(output)
                rosys.notify('Resetting ESP: failed', 'negative')

    def _convert_flash_params(self, flash_params: list[str]) -> list[str]:
        """Until the deprecation of the flash.py script, we need to convert the flash_params to espresso parameters."""
        espresso_parameters = []
        self.log.debug('flash_params: %s', flash_params)
        if 'orin' in flash_params:
            espresso_parameters.extend(['--jetson', 'orin'])
        elif 'xavier' in flash_params:
            espresso_parameters.extend(['--jetson', 'xavier'])
        elif 'nano' in flash_params:
            espresso_parameters.extend(['--jetson', 'nano'])
        if 'nand' in flash_params:
            espresso_parameters.append('--nand')
        if 'orin' in flash_params and 'v05' not in flash_params:
            espresso_parameters.append('--swap_pins')
        self.log.debug('espresso_parameters: %s', espresso_parameters)
        return espresso_parameters

    def __del__(self) -> None:
        self.communication.disconnect()

    def __repr__(self) -> str:
        return f'<RobotBrain {self.communication}>'


class EspNotReadyException(Exception):
    """Raised when trying to use the ESP before it is ready."""


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
