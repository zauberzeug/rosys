import socket
import struct
import logging

class MotecSettingsInterface:
    def __init__(self, ip: str, port: int):
        self.ip = ip
        self.port = port

        self.log = logging.getLogger("MotecSettingsInterface")
        self.log.setLevel(logging.DEBUG)

        self._connect()
    
    def __del__(self):
        self._disconnect()

    def _connect(self):
        self.s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.s.connect((self.ip, self.port))
        self.s.setblocking(True)

        self.log.debug("Connected to %s:%d", self.ip, self.port)

    def _disconnect(self):
        self.s.close()

    def _construct_message(self, command_id: int, value_id: int, values: 'list[int]'):
        n_value_bytes = len(values)
        if n_value_bytes > 8:
            raise ValueError("Too many values provided")
        
        v = [0]*8
        for i, value in enumerate(values):
            v[i] = value
        
        return struct.pack('B'*14, 48, 2, command_id, n_value_bytes+1, value_id, 0, *v)

    def _get_value(self, value_id: int) -> bytearray:
        message = self._construct_message(70, value_id, [])
        self.log.debug("Sending message [get]: %s", message)
        self.s.send(message)

        data = self.s.recv(1024)
        values = struct.unpack('B'*12, data)[6:]
        self.log.debug("Received message [get]: %s", values)
        return bytearray(values)
    
    def _set_value(self, value_id: int, values: list[int] | int):
        if isinstance(values, int):
            values = [values]

        message = self._construct_message(38, value_id, values)
        self.log.debug("Sending message [set]: %s", message)
        self.s.send(message)

    def get_fps(self):
        return self._get_value(178)[0]

    def set_fps(self, fps: int):
        self._set_value(178, fps)

    def get_stream_compression(self):
        return self._get_value(177)[0]
    
    def set_stream_compression(self, level: int):
        if level < 1 or level > 4:
            raise ValueError("Compression level must be between 1 and 4")

        self._set_value(177, level)

    def get_stream_resolution(self) -> tuple[int, int]:
        wmsb, wlsb, hmsb, hlsb = self._get_value(179)[:4]
        return (wmsb << 8) + wlsb, (hmsb << 8) + hlsb
    
    def set_stream_resolution(self, width: int, height: int):
        wmsb = width >> 8
        wlsb = width & 0xFF
        hmsb = height >> 8
        hlsb = height & 0xFF
        self._set_value(179, [wmsb, wlsb, hmsb, hlsb])

    def get_stream_port(self) -> int:
        pmsb, plsb = self._get_value(200)[:2]
        return (pmsb << 8) + plsb
    
    def set_stream_port(self, port: int):
        pmsb = port >> 8
        plsb = port & 0xFF
        self._set_value(200, [pmsb, plsb])

    def request_image(self):
        """
        Requests the camera to send an image if it is considered to only send images on request
        """
        message = self._construct_message(40, 185, [0]*8)
        self.s.send(message)


