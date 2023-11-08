import json
import time
from dataclasses import dataclass
from typing import Any, Optional

import requests


@dataclass
class JovisionCameraSettings:
    """
    Dataclass that holds the settings of an RTSP camera
    """
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
            'rcMode': self.bitrate_control
        }


class JovisionInterface:
    """
    Interface that communicates with the RTSP camera settings mimicking the webpage requests
    """

    def __init__(self, ip: str) -> None:
        self.ip = ip

    def _get_settings_url(self) -> str:
        return f'http://{self.ip}/cgi-bin/jvsweb.cgi'

    def _get_headers(self) -> dict[str, str]:
        return {'Cookie': 'passwdRule=1; username=admin; password=admin;'}

    def _send_settings(self, settings: list[JovisionCameraSettings]) -> None:
        """
        Set the settings for all streams
        """
        url = self._get_settings_url()
        n_streams = len(settings)
        streams: list[dict] = []
        for i, setting in enumerate(settings):
            parameter_dict = setting.to_dict()
            parameter_dict['smartencode'] = 'open' if i < n_streams - 1 else 'close'
            streams.append(parameter_dict)

        cmd = {
            'method': 'stream_set_params',
            'user': {
                'name': 'admin',
                'digest': '479ead4555b49227dc8812d06970c652',
            },
            'param': {
                'channelid': 0,
                'streams': streams,
            },
        }
        params = {
            'cmd': json.dumps(cmd),
            '_': int(time.time() * 1000),  # current time as a timestamp
        }
        headers = self._get_headers()
        requests.get(url, params=params, headers=headers, timeout=1)  # type: ignore

    def set_fps(self, stream_id, fps) -> None:
        current_settings = self.get_current_settings()
        for settings in current_settings:
            if settings.stream_id == stream_id:
                settings.fps = fps
                break
        self._send_settings(current_settings)

    def get_fps(self, stream_id) -> Optional[int]:
        current_settings = self.get_current_settings()
        for settings in current_settings:
            if settings.stream_id == stream_id:
                return settings.fps

        return None

    def get_current_settings(self) -> list[JovisionCameraSettings]:
        url = self._get_settings_url()
        cmd = {
            'method': 'stream_get_params',
            'user': {
                'name': 'admin',
                'digest': "d065077d3e61cb56d45bd2dd28b83842",
            },
            'param': {
                'channelid': 0,
            },
        }
        params = {
            'cmd': json.dumps(cmd),
            '_': int(time.time() * 1000),
        }
        headers = self._get_headers()

        response = requests.get(url, params=params, headers=headers, timeout=1)  # type: ignore

        settings = []
        streams = response.json()['result']['streams']
        for stream in streams:
            stream_settings = JovisionCameraSettings(
                codec=stream['venctype'],
                fps=stream['frameRate'],
                quality=stream['quality'],
                resolution=(stream['width'], stream['height']),
                bitrate_control=stream['rcMode'],
                bitrate=stream['bitRate'],
                stream_id=stream['streamid'],
                channel_id=stream['channelid'],
            )
            settings.append(stream_settings)

        return settings

    def get_parameter_ranges(self):
        raise NotImplementedError
        # pylint: disable=unreachable
        url = self._get_settings_url()
        cmd = {
            'method': 'stream_get_all_ability',
            'user': {
                'name': 'admin',
                'digest': "8d5985ea2b9994aacb6cb3e3f826aae5"
            },
            'param': {
                'channelid': 0
            }
        }
        params = {
            'cmd': json.dumps(cmd),
            '_': time.time() * 1000
        }
        headers = self._get_headers()

        response = requests.get(url, params=params, headers=headers, timeout=1)

        for stream_id, stream in enumerate(response.json()['result']['all']):
            print(f'stream {stream_id}')
            resolutions = stream['resolutions']
            for resolution in resolutions:
                width = resolution['width']
                height = resolution['height']
                b_default = resolution['bDefault']
                max_kbps = resolution['maxKbps']
                min_kbps = resolution['minKbps']
                def_kbps = resolution['defKbps']
                max_fr = resolution['maxFr']
                min_fr = resolution['minFr']
                def_fr = resolution['defFr']
            max_quality = stream['maxQuality']
            min_quality = stream['minQuality']
            max_ngop = stream['maxNGOP']
            min_ngop = stream['minNGOP']
            b_support_h265 = stream['bSupportH265']
            b_support_smart_enc = stream['bSupportSmartEnc']
