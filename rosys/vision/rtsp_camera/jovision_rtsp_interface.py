import json
import time
from dataclasses import dataclass
from typing import Any

import httpx
from httpx import QueryParams


@dataclass
class JovisionCameraSettings:
    """Dataclass that holds the settings of an RTSP camera"""
    codec: str
    fps: int
    quality: str
    resolution: tuple[int, int]
    bitrate_control: str
    bitrate: int
    stream_id: int
    channel_id: int = 0

    def to_dict(self) -> dict[str, Any]:
        return {
            'channelid': self.channel_id,
            'streamid': self.stream_id,
            'venctype': self.codec,
            'width': self.resolution[0],
            'height': self.resolution[1],
            'frameRate': self.fps,
            'bitRate': self.bitrate,
            'ngop_s': 100,
            'quality': self.quality,
            'rcMode': self.bitrate_control,
        }


class JovisionInterface:
    """Interface that communicates with the RTSP camera settings mimicking the webpage requests"""

    def __init__(self, ip: str) -> None:
        self.ip = ip

    @property
    def settings_url(self) -> str:
        return f'http://{self.ip}/cgi-bin/jvsweb.cgi'

    async def _send_settings(self, settings: list[JovisionCameraSettings]) -> None:
        """
        Set the settings for all streams
        """
        streams: list[dict] = []
        for i, setting in enumerate(settings):
            parameter_dict = setting.to_dict()
            parameter_dict['smartencode'] = 'open' if i < len(settings) - 1 else 'close'
            streams.append(parameter_dict)

        cmd = {
            'method': 'stream_set_params',
            'user': {
                'name': 'admin',
                'digest': '479ead4555b49227dc8812d06970c652',
            },
            'param': {
                'streams': streams,
            },
        }
        params = QueryParams(
            cmd=json.dumps(cmd),
            _=int(time.time() * 1000),  # current time as a timestamp
        )
        async with httpx.AsyncClient() as client:
            await client.get(self.settings_url, params=params)

    async def set_fps(self, stream_id: int, fps: int) -> None:
        current_settings = await self.get_current_settings()
        for settings in current_settings:
            if settings.stream_id == stream_id:
                settings.fps = int(fps)
                break
        await self._send_settings(current_settings)

    async def get_fps(self, stream_id: int) -> int | None:
        current_settings = await self.get_current_settings()
        for settings in current_settings:
            if settings.stream_id == stream_id:
                return settings.fps

        return None

    async def get_bitrate(self, stream_id: int) -> int | None:
        current_settings = await self.get_current_settings()
        for settings in current_settings:
            if settings.stream_id == stream_id:
                return settings.bitrate

        return None

    async def set_bitrate(self, stream_id: int, bitrate: int) -> None:
        current_settings = await self.get_current_settings()
        for settings in current_settings:
            if settings.stream_id == stream_id:
                settings.bitrate = int(bitrate)
                break
        await self._send_settings(current_settings)

    async def get_current_settings(self) -> list[JovisionCameraSettings]:
        cmd = {
            'method': 'stream_get_params',
            'user': {
                'name': 'admin',
                'digest': 'd065077d3e61cb56d45bd2dd28b83842',
            },
            'param': {
                'channelid': 0,
            },
        }
        params = QueryParams(
            cmd=json.dumps(cmd),
            _=int(time.time() * 1000),
        )
        async with httpx.AsyncClient() as client:
            response = await client.get(self.settings_url, params=params)

        return [
            JovisionCameraSettings(
                codec=stream['venctype'],
                fps=stream['frameRate'],
                quality=stream['quality'],
                resolution=(stream['width'], stream['height']),
                bitrate_control=stream['rcMode'],
                bitrate=stream['bitRate'],
                stream_id=stream['streamid'],
                channel_id=stream['channelid'],
            )
            for stream in response.json()['result']['streams']
        ]

    async def get_parameter_ranges(self):
        raise NotImplementedError

        # cmd = {
        #     'method': 'stream_get_all_ability',
        #     'user': {
        #         'name': 'admin',
        #         'digest': '8d5985ea2b9994aacb6cb3e3f826aae5'
        #     },
        #     'param': {
        #         'channelid': 0,
        #     }
        # }
        # params = {
        #     'cmd': json.dumps(cmd),
        #     '_': time.time() * 1000,
        # }
        # async with httpx.AsyncClient() as client:
        #     response = await client.get(self.settings_url, params=params)

        # for stream_id, stream in enumerate(response.json()['result']['all']):
        #     print(f'stream {stream_id}')
        #     resolutions = stream['resolutions']
        #     for resolution in resolutions:
        #         width = resolution['width']
        #         height = resolution['height']
        #         b_default = resolution['bDefault']
        #         max_kbps = resolution['maxKbps']
        #         min_kbps = resolution['minKbps']
        #         def_kbps = resolution['defKbps']
        #         max_fr = resolution['maxFr']
        #         min_fr = resolution['minFr']
        #         def_fr = resolution['defFr']
        #     max_quality = stream['maxQuality']
        #     min_quality = stream['minQuality']
        #     max_ngop = stream['maxNGOP']
        #     min_ngop = stream['minNGOP']
        #     b_support_h265 = stream['bSupportH265']
        #     b_support_smart_enc = stream['bSupportSmartEnc']
