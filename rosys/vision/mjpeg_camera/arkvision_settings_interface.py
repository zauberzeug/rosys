import logging
from typing import Any

import httpx

# The achievable frame rates depend on the sensor mode, which fixes a "base" frame rate.
# (frequency, mode) -> base fps.  frequency: 0 = 50 Hz, 1 = 60 Hz.  See /imageSensorMode in the REST spec.
SENSOR_MODE_BASE_FPS: dict[tuple[int, int], int] = {
    (0, 0): 25, (0, 1): 12, (0, 2): 25, (0, 3): 50, (0, 4): 25, (0, 5): 50,
    (1, 0): 30, (1, 1): 15, (1, 2): 30, (1, 3): 60, (1, 4): 30, (1, 5): 60,
}

# base fps -> ordered list of selectable frame rates; the list index is the `framerate` enum value.
FRAMERATE_DIVISIONS: dict[int, list[int]] = {
    12: [12, 6, 4, 2, 1],
    15: [15, 5, 3, 1],
    25: [25, 5, 1],
    30: [30, 15, 10, 6, 5, 3, 2, 1],
    50: [50, 25, 10, 5, 2, 1],
    60: [60, 30, 20, 15, 12, 10, 6, 5, 4, 3, 2, 1],
}

# `resolution` enum value -> (width, height) for the MJPEG stream (see /streamMjpeg in the REST spec).
MJPEG_RESOLUTIONS: list[tuple[int, int]] = [
    (1920, 1080), (1280, 720), (960, 544), (800, 600), (800, 480),
    (704, 400), (640, 480), (640, 360), (480, 360),
]


def nearest_index(values: list[int], target: int) -> int:
    """Index of the entry in `values` closest to `target`; ties resolve to the lower index."""
    return min(range(len(values)), key=lambda i: (abs(values[i] - target), i))


class ArkVisionSettingsInterface:
    """Reads and writes ArkCam Basic+ settings through its REST API (``http://<ip>/rest/...``).

    The REST server has no authentication: a GET reads a settings group, a POST writes (partial) JSON.
    Getters return ``None`` on a REST error so the caller can keep the previously cached value
    (``ConfigurableCamera`` ignores ``None`` values).
    """

    def __init__(self, ip: str) -> None:
        self.ip = ip
        self.log = logging.getLogger('rosys.vision.mjpeg_camera.arkvision_settings_interface')

    @property
    def base_url(self) -> str:
        return f'http://{self.ip}/rest'

    async def _get(self, endpoint: str) -> dict[str, Any] | None:
        try:
            async with httpx.AsyncClient() as client:
                response = await client.get(f'{self.base_url}/{endpoint}', timeout=5)
            response.raise_for_status()
            return response.json()
        except (httpx.HTTPError, ValueError) as e:
            self.log.warning('could not read "%s": %s', endpoint, e)
            return None

    async def _post(self, endpoint: str, data: dict[str, Any]) -> None:
        try:
            async with httpx.AsyncClient() as client:
                response = await client.post(f'{self.base_url}/{endpoint}', json=data, timeout=5)
            response.raise_for_status()
        except httpx.HTTPError as e:
            self.log.warning('could not write "%s" (%s): %s', endpoint, data, e)

    async def enable_http_mjpeg(self) -> None:
        """Ensure the MJPEG-over-HTTP stream (port 81) is enabled (``mJpegStreamType`` = 2 = HTTP)."""
        common = await self._get('streamCommon')
        if common is None or common.get('mJpegStreamType') == 2:
            return
        self.log.info('enabling MJPEG-over-HTTP stream (mJpegStreamType=2)')
        await self._post('streamCommon', {'mJpegStreamType': 2})

    async def _base_fps(self) -> int | None:
        sensor_mode = await self._get('imageSensorMode')
        if sensor_mode is None:
            return None
        key = (sensor_mode.get('frequency'), sensor_mode.get('mode'))
        base_fps = SENSOR_MODE_BASE_FPS.get(key)  # type: ignore[arg-type]
        if base_fps is None:
            self.log.warning('unknown sensor mode %s', key)
        return base_fps

    async def get_fps(self) -> int | None:
        base_fps = await self._base_fps()
        stream = await self._get('streamMjpeg')
        if base_fps is None or stream is None:
            return None
        divisions = FRAMERATE_DIVISIONS[base_fps]
        index = max(0, min(int(stream.get('framerate', 0)), len(divisions) - 1))
        return divisions[index]

    async def set_fps(self, fps: int) -> None:
        base_fps = await self._base_fps()
        if base_fps is None:
            return
        index = nearest_index(FRAMERATE_DIVISIONS[base_fps], fps)
        await self._post('streamMjpeg', {'framerate': index})

    async def get_resolution(self) -> tuple[int, int] | None:
        stream = await self._get('streamMjpeg')
        if stream is None:
            return None
        index = int(stream.get('resolution', 0))
        if not 0 <= index < len(MJPEG_RESOLUTIONS):
            return None
        return MJPEG_RESOLUTIONS[index]

    async def set_resolution(self, width: int, height: int) -> None:
        index = min(range(len(MJPEG_RESOLUTIONS)),
                    key=lambda i: abs(MJPEG_RESOLUTIONS[i][0] - width) + abs(MJPEG_RESOLUTIONS[i][1] - height))
        await self._post('streamMjpeg', {'resolution': index})

    async def get_mirrored(self) -> bool | None:
        orientation = await self._get('imageOrientation')
        if orientation is None:
            return None
        return bool(orientation.get('mirror', False))

    async def set_mirrored(self, mirrored: bool) -> None:
        await self._post('imageOrientation', {'mirror': mirrored})
