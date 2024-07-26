import re
from dataclasses import dataclass

from .mjpeg_device import MjpegDevice
from .vendors import VendorType, mac_to_vendor


@dataclass(slots=True, kw_only=True)
class AxisSettings:
    fps: int
    resolution: tuple[int, int]
    mirrored: bool

    def __post_init__(self):
        if self.fps <= 0:
            raise ValueError(f'fps must be positive. Got {self.fps}')
        if self.resolution[0] <= 0 or self.resolution[1] <= 0:
            raise ValueError(f'resolution must be positive. Got {self.resolution}')
        if not isinstance(self.mirrored, bool):
            raise ValueError(f'mirrored must be a boolean. Got {self.mirrored}')


class AxisMjpegDevice(MjpegDevice):
    def __init__(self, mac: str, ip: str, *,
                 index: int | None = None,
                 username: str | None = None,
                 password: str | None = None) -> None:
        super().__init__(mac, ip,
                         index=index,
                         username=username, password=password)

        self.axis_settings = AxisSettings(fps=6, resolution=(640, 480), mirrored=False)

        vendor = mac_to_vendor(mac)
        if vendor != VendorType.AXIS:
            raise ValueError(f'AxisMjpegDevice can only be used with AXIS devices. Got {vendor} for mac="{mac}"')

    async def get_fps(self) -> int:
        return self.axis_settings.fps

    async def set_fps(self, fps: int) -> None:
        self.url = re.sub(r'fps=\d+', f'fps={fps}', self.url)
        if 'fps=' not in self.url:
            self.url += f'&fps={fps}'
        self.restart_capture()

    async def get_resolution(self) -> tuple[int, int]:
        return self.axis_settings.resolution

    async def set_resolution(self, width: int, height: int) -> None:
        self.url = re.sub(r'resolution=\d+x\d+', f'resolution={width}x{height}', self.url)
        if 'resolution=' not in self.url:
            self.url += f'&resolution={width}x{height}'
        self.restart_capture()

    async def get_mirrored(self) -> bool:
        return self.axis_settings.mirrored

    async def set_mirrored(self, mirrored: bool) -> None:
        value = 1 if mirrored else 0
        self.url = re.sub(r'mirror=\d', f'mirror={value}', self.url)
        if 'mirror=' not in self.url:
            self.url += f'&mirror={value}'
        self.restart_capture()
