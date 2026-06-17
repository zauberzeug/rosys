"""Settings interface for OpenIPC cameras running the divinus streamer.

OpenIPC cameras flashed with the divinus streamer
(https://github.com/zauberzeug/divinus) expose an HTTP control API. This
interface drives that API so the camera parameters RoSys already models
(fps, bitrate, resolution, mirroring, ...) are actually applied on the device
instead of being silently dropped.

Endpoint mapping (see divinus ``doc/openapi.yaml``):

* ``RtspCamera`` streams the H.26x encoder -> ``/api/mp4``   (fps, bitrate)
* ``MjpegCamera`` streams the MJPEG encoder -> ``/api/mjpeg`` (fps, resolution)
* mirroring is an ISP setting shared by both -> ``/api/isp``

Every ``/api/<x>`` endpoint reads the current state when called without
parameters and writes the supplied ones (returning the new state) otherwise.
"""
from typing import Any

import httpx

# divinus documents the /api/mp4 and /api/mjpeg bitrate unit inconsistently (kbps in
# config.md, bit/s in endpoints.md). RoSys models bitrate in kbps, so the value is
# passed through unchanged; verify against a live camera and adjust here if needed.

REQUEST_TIMEOUT = 5.0


class OpenIpcSettingsInterface:
    """Communicates with the divinus HTTP API of an OpenIPC camera."""

    def __init__(self, ip: str, *, username: str | None = None, password: str | None = None) -> None:
        self.ip = ip
        self._auth = httpx.BasicAuth(username, password) if username and password else None

    @property
    def base_url(self) -> str:
        return f'http://{self.ip}'

    async def _request(self, path: str, **params: Any) -> dict[str, Any]:
        params = {key: value for key, value in params.items() if value is not None}
        async with httpx.AsyncClient(auth=self._auth, timeout=REQUEST_TIMEOUT) as client:
            response = await client.get(f'{self.base_url}{path}', params=params)
            response.raise_for_status()
            return response.json()

    # --- raw endpoints -------------------------------------------------------

    async def get_mp4(self) -> dict[str, Any]:
        return await self._request('/api/mp4')

    async def get_mjpeg(self) -> dict[str, Any]:
        return await self._request('/api/mjpeg')

    async def get_isp(self) -> dict[str, Any]:
        return await self._request('/api/isp')

    # --- RTSP / H.26x stream (RtspDevice settings-interface contract) ---------
    # ``stream_id`` is part of the contract shared with JovisionInterface; divinus
    # exposes a single H.26x encoder, so it is accepted and ignored here.

    async def set_fps(self, stream_id: int, fps: int) -> None:  # pylint: disable=unused-argument
        await self._request('/api/mp4', fps=int(fps))

    async def get_fps(self, stream_id: int) -> int | None:  # pylint: disable=unused-argument
        return (await self.get_mp4()).get('fps')

    async def set_bitrate(self, stream_id: int, bitrate: int) -> None:  # pylint: disable=unused-argument
        await self._request('/api/mp4', bitrate=int(bitrate))

    async def get_bitrate(self, stream_id: int) -> int | None:  # pylint: disable=unused-argument
        return (await self.get_mp4()).get('bitrate')

    # --- MJPEG stream + ISP (OpenIpcMjpegDevice) -----------------------------

    async def set_mjpeg_fps(self, fps: int) -> None:
        await self._request('/api/mjpeg', fps=int(fps))

    async def get_mjpeg_fps(self) -> int | None:
        return (await self.get_mjpeg()).get('fps')

    async def set_resolution(self, width: int, height: int) -> None:
        await self._request('/api/mjpeg', width=int(width), height=int(height))

    async def get_resolution(self) -> tuple[int, int]:
        mjpeg = await self.get_mjpeg()
        return int(mjpeg['width']), int(mjpeg['height'])

    async def set_mirrored(self, mirrored: bool) -> None:
        await self._request('/api/isp', mirror=mirrored)

    async def get_mirrored(self) -> bool | None:
        return (await self.get_isp()).get('mirror')
