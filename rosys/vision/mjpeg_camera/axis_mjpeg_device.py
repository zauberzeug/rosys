from collections.abc import Awaitable, Callable
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
    """MJPEG device for AXIS cameras, which take fps, resolution and mirroring as stream URL parameters."""

    def __init__(self, mac: str, ip: str | None = None, *,
                 index: int | None = None,
                 username: str | None = None,
                 password: str | None = None,
                 on_new_image_data: Callable[[bytes, float], Awaitable | None],
                 on_connect: Callable[[], Awaitable | None] | None = None) -> None:
        vendor = mac_to_vendor(mac)
        if vendor != VendorType.AXIS:
            raise ValueError(f'AxisMjpegDevice can only be used with AXIS devices. Got {vendor} for mac="{mac}"')

        self.axis_settings = AxisSettings(fps=6, resolution=(640, 480), mirrored=False)

        super().__init__(mac, ip, index=index, username=username, password=password,
                         on_new_image_data=on_new_image_data, on_connect=on_connect)

    @property
    def url(self) -> str | None:
        url = super().url
        if url is None:
            return None
        width, height = self.axis_settings.resolution
        query = '&'.join([
            f'fps={self.axis_settings.fps}',
            f'resolution={width}x{height}',
            f'mirror={1 if self.axis_settings.mirrored else 0}',
        ])
        separator = '' if url.endswith('?') else '&'
        return f'{url}{separator}{query}'

    async def get_fps(self) -> int:
        return self.axis_settings.fps

    async def set_fps(self, fps: int) -> None:
        self.axis_settings.fps = fps
        self.restart_capture()

    async def get_resolution(self) -> tuple[int, int]:
        return self.axis_settings.resolution

    async def set_resolution(self, width: int, height: int) -> None:
        self.axis_settings.resolution = (width, height)
        self.restart_capture()

    async def get_mirrored(self) -> bool:
        return self.axis_settings.mirrored

    async def set_mirrored(self, mirrored: bool) -> None:
        self.axis_settings.mirrored = mirrored
        self.restart_capture()
