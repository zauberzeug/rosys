import json
import time
from dataclasses import dataclass
from typing import Optional

import requests


@dataclass
class JovisionCameraSettings:
    """
    Dataclass that holds the settings of a rtsp camera
    """
    codec: str
    fps: int
    quality: str
    resolution: tuple[int, int]
    bitrate_control: str
    bitrate: int
    stream_id: int
    channel_id: int = 0

    def to_dict(self):
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
    Interface that communicates with the rtsp camera settings mimicking the webpage requests
    """
    N_STREAMS = 2
    IP: str

    def __init__(self, ip: str) -> None:
        self.IP = ip

    def get_settings_url(self):
        return f'http://{self.IP}/cgi-bin/jvsweb.cgi'

    def get_headers(self):
        return {'Cookie': 'passwdRule=1; username=admin; password=admin;'}

    def send_settings(self, settings: list[JovisionCameraSettings]):
        """
        Set the settings for all streams
        """
        url = self.get_settings_url()
        n_streams = len(settings)
        cmd = {
            'method': 'stream_set_params',
            'user': {
                'name': 'admin',
                'digest': '479ead4555b49227dc8812d06970c652'
            },
            'param': {
                'channelid': 0,
                'streams': []
            }
        }
        for i, setting in enumerate(settings):
            parameter_dict = setting.to_dict()
            if i < n_streams - 1:
                parameter_dict['smartencode'] = 'open'
            else:
                parameter_dict['smartencode'] = 'close'
            cmd['param']['streams'].append(parameter_dict)

        params = {
            'cmd': json.dumps(cmd),
            '_': int(time.time() * 1000)  # current time as a timestamp
        }
        headers = self.get_headers()
        print(url)
        print(params)
        print(headers)

        response = requests.get(url, params=params, headers=headers, timeout=1)

        print(response.status_code)
        print(response.text)

    def set_fps(self, stream_id, fps):
        current_settings = self.get_current_settings()
        for settings in current_settings:
            if settings.stream_id == stream_id:
                print(f'Changing fps from {settings.fps} to {fps}')
                settings.fps = fps
                break
        self.send_settings(current_settings)

    def get_fps(self, stream_id) -> Optional[int]:
        current_settings = self.get_current_settings()
        for settings in current_settings:
            if settings.stream_id == stream_id:
                return settings.fps

        return None

    def get_current_settings(self) -> list[JovisionCameraSettings]:
        """
        Get the current settings of the camera for all streams
        """
        url = self.get_settings_url()
        cmd = {
            'method': 'stream_get_params',
            'user': {
                'name': 'admin',
                'digest': "d065077d3e61cb56d45bd2dd28b83842"
            },
            'param': {
                'channelid': 0
            }
        }
        params = {
            'cmd': json.dumps(cmd),
            '_': int(time.time() * 1000)
        }
        headers = self.get_headers()

        response = requests.get(url, params=params, headers=headers, timeout=1)

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
                channel_id=stream['channelid']
            )
            settings.append(stream_settings)

        return settings

    def get_parameter_ranges(self):
       # TODO do something with the values
        url = self.get_settings_url()
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
        headers = self.get_headers()

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


if __name__ == '__main__':
    ip = '192.168.1.242'
    rtsp_interface = JovisionInterface(ip)
    settings = rtsp_interface.get_current_settings()
    print(settings)

    rtsp_interface.get_parameter_ranges()

    rtsp_interface.set_fps(0, 20)
    settings = rtsp_interface.get_current_settings()
    print(settings)
