from ..arkvision_rest_client import ArkVisionRestClient, nearest_index

# `bitrate` enum value -> kbit/s for the H.264 stream (see /streamH264 in the REST spec).
H264_BITRATES_KBIT: list[int] = [
    128, 256, 512, 1024, 2048, 3072, 4096, 5120, 6144, 7168, 8192, 9216,
    10240, 11264, 12288, 13312, 14336, 15360, 16384, 17408, 18432, 19456, 20480,
]


class ArkVisionRtspInterface(ArkVisionRestClient):
    """Reads and writes ArkCam Basic+ H.264 stream settings (fps, bitrate) via REST.

    The ArkCam exposes a single H.264 stream, so the ``stream_id`` argument (kept for interface
    compatibility with the other RTSP settings interfaces) is ignored.
    """

    async def get_fps(self, stream_id: int) -> int | None:  # pylint: disable=unused-argument
        return await self.get_framerate('streamH264')

    async def set_fps(self, stream_id: int, fps: int) -> None:  # pylint: disable=unused-argument
        await self.set_framerate('streamH264', fps)

    async def get_bitrate(self, stream_id: int) -> int | None:  # pylint: disable=unused-argument
        stream = await self._get('streamH264')
        if stream is None:
            return None
        index = max(0, min(int(stream.get('bitrate', 0)), len(H264_BITRATES_KBIT) - 1))
        return H264_BITRATES_KBIT[index]

    async def set_bitrate(self, stream_id: int, bitrate: int) -> None:  # pylint: disable=unused-argument
        index = nearest_index(H264_BITRATES_KBIT, bitrate)
        await self._post('streamH264', {'bitrate': index})
