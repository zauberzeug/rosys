from ..arkvision_rest_client import ArkVisionRestClient

# `resolution` enum value -> (width, height) for the MJPEG stream (see /streamMjpeg in the REST spec).
MJPEG_RESOLUTIONS: list[tuple[int, int]] = [
    (1920, 1080), (1280, 720), (960, 544), (800, 600), (800, 480),
    (704, 400), (640, 480), (640, 360), (480, 360),
]


class ArkVisionSettingsInterface(ArkVisionRestClient):
    """Reads and writes ArkCam Basic+ MJPEG stream settings (fps, resolution, mirror) via REST."""

    async def enable_http_mjpeg(self) -> None:
        """Ensure the MJPEG-over-HTTP stream is enabled (``mJpegStreamType`` = 2 = HTTP)."""
        common = await self._get('streamCommon')
        if common is None or common.get('mJpegStreamType') == 2:
            return
        self.log.info('enabling MJPEG-over-HTTP stream (mJpegStreamType=2)')
        await self._post('streamCommon', {'mJpegStreamType': 2})

    async def get_fps(self) -> int | None:
        return await self.get_framerate('streamMjpeg')

    async def set_fps(self, fps: int) -> None:
        await self.set_framerate('streamMjpeg', fps)

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
