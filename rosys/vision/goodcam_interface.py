"""Interface for GoodCam devices using the REST API.

GoodCam devices provide a REST API with HTTP Digest authentication.
See https://goodcam.github.io/goodcam-api/ for full documentation.
"""

from typing import Any

import httpx


def _stream_id_to_name(stream_id: int) -> str:
    """Map numeric stream ID to GoodCam stream name."""
    return 'secondary' if stream_id else 'primary'


class GoodCamInterface:
    """Interface for getting and setting GoodCam camera parameters via the REST API."""

    def __init__(self, ip: str, *, username: str = 'root', password: str = 'Adminadmin') -> None:
        self._ip = ip
        self._auth = httpx.DigestAuth(username, password)
        self._base_url = f'http://{ip}/api/v1'

    async def _get(self, path: str) -> dict[str, Any]:
        async with httpx.AsyncClient() as client:
            response = await client.get(f'{self._base_url}{path}', auth=self._auth)
            response.raise_for_status()
            return response.json()

    async def _put(self, path: str, data: dict[str, Any]) -> dict[str, Any]:
        async with httpx.AsyncClient() as client:
            response = await client.put(f'{self._base_url}{path}', json=data, auth=self._auth)
            response.raise_for_status()
            return response.json()

    async def get_stream_settings(self, stream_id: int) -> dict[str, Any]:
        """Get current settings for the given stream."""
        stream_name = _stream_id_to_name(stream_id)
        return await self._get(f'/streams/{stream_name}/')

    async def _update_stream_video(self, stream_id: int, **kwargs: Any) -> None:
        """Update video settings for the given stream. Only provided fields are changed."""
        settings = await self.get_stream_settings(stream_id)
        video = dict(settings.get('video', {}))
        video.update(kwargs)
        stream_name = _stream_id_to_name(stream_id)
        await self._put(f'/streams/{stream_name}/', {'video': video})

    async def get_fps(self, stream_id: int) -> int | None:
        """Get frames per second for the given stream."""
        settings = await self.get_stream_settings(stream_id)
        video = settings.get('video')
        if video is None:
            return None
        return video.get('fps')

    async def set_fps(self, stream_id: int, fps: int) -> None:
        """Set frames per second for the given stream."""
        await self._update_stream_video(stream_id, fps=fps)

    async def get_quality(self, stream_id: int) -> int | None:
        """Get compression quality (0-100) for the given stream."""
        settings = await self.get_stream_settings(stream_id)
        video = settings.get('video')
        if video is None:
            return None
        bitrate_config = video.get('bitrate')
        if bitrate_config is None:
            return None
        return bitrate_config.get('quality')

    async def set_quality(self, stream_id: int, quality: int) -> None:
        """Set compression quality (0-100) for the given stream."""
        settings = await self.get_stream_settings(stream_id)
        video = settings.get('video', {})
        bitrate_config = dict(video.get('bitrate', {}))
        bitrate_config['quality'] = quality
        await self._update_stream_video(stream_id, bitrate=bitrate_config)

    async def get_bitrate(self, stream_id: int) -> int | None:
        """Get bitrate in kbps for the given stream."""
        settings = await self.get_stream_settings(stream_id)
        video = settings.get('video')
        if video is None:
            return None
        bitrate_config = video.get('bitrate')
        if bitrate_config is None:
            return None
        mode = bitrate_config.get('mode')
        if mode == 'cbr':
            return bitrate_config.get('bitrate')
        return bitrate_config.get('max_bitrate')

    async def set_bitrate(self, stream_id: int, bitrate: int) -> None:
        """Set bitrate in kbps for the given stream."""
        settings = await self.get_stream_settings(stream_id)
        video = settings.get('video', {})
        bitrate_config = dict(video.get('bitrate', {}))
        mode = bitrate_config.get('mode', 'vbr')
        if mode == 'cbr':
            bitrate_config['bitrate'] = bitrate
        else:
            bitrate_config['max_bitrate'] = bitrate
        await self._update_stream_video(stream_id, bitrate=bitrate_config)

    async def get_resolution(self, stream_id: int) -> tuple[int, int] | None:
        """Get (width, height) for the given stream."""
        settings = await self.get_stream_settings(stream_id)
        video = settings.get('video')
        if video is None:
            return None
        width = video.get('width')
        height = video.get('height')
        if width is None or height is None:
            return None
        return (width, height)

    async def set_resolution(self, stream_id: int, width: int, height: int) -> None:
        """Set resolution for the given stream."""
        await self._update_stream_video(stream_id, width=width, height=height)

    async def get_stream_limits(self, stream_id: int) -> dict[str, Any]:
        """Get valid parameter ranges for the given stream."""
        stream_name = _stream_id_to_name(stream_id)
        return await self._get(f'/streams/{stream_name}/limits/')
