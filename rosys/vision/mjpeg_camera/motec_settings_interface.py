import asyncio
import logging
import struct

from typing_extensions import Self


class AsyncTcpClient:
    """Async TCP client wrapper for sending and receiving messages to/from a server."""

    def __init__(self, ip: str, port: int) -> None:
        self.ip = ip
        self.port = port
        self.reader: asyncio.StreamReader | None = None
        self.writer: asyncio.StreamWriter | None = None

    async def __aenter__(self) -> Self:
        self.reader, self.writer = await asyncio.open_connection(self.ip, self.port)
        return self

    async def __aexit__(self, *_) -> None:
        assert self.writer is not None
        self.writer.close()
        await self.writer.wait_closed()

    async def read(self, n_bytes: int, timeout_s: float = 3) -> bytes:
        """Read n_bytes from the server with a timeout.

        :param n_bytes: Number of bytes to read
        :param timeout_s: Timeout in seconds
        :return: Bytes read from the server

        :raises TimeoutError: If no response is received within the timeout
        :raises RuntimeError: If the response length is not equal to n_bytes
        """
        assert self.reader is not None
        data = await asyncio.wait_for(self.reader.read(n_bytes), timeout_s)
        if not data:
            raise TimeoutError('No response received within timeout')
        if not len(data) == n_bytes:
            raise RuntimeError(f'Received response with unexpected length {len(data)}')

        return data

    async def write(self, message: bytes) -> None:
        """Write a message to the server.

        :param message: Message to send
        """
        assert self.writer is not None
        self.writer.write(message)


class MotecSettingsInterface:

    def __init__(self, ip: str, port: int) -> None:
        self.ip = ip
        self.port = port

        self.log = logging.getLogger('MotecSettingsInterface')

        self.cooldown = 0.2
        self.cooldown_time = 0.0

        self.event_loop = asyncio.get_event_loop()

    def _construct_message(self, command_id: int, value_id: int, values: list[int]) -> bytes:
        n_value_bytes = len(values)
        if n_value_bytes > 6:
            raise ValueError('Too many values provided')

        v = [0] * 6
        for i, value in enumerate(values):
            v[i] = value

        return struct.pack('B' * 12, 48, 2, command_id, n_value_bytes + 1, value_id, 0, *v)

    async def _get_value(self, value_id: int, timeout_s: float = 3) -> bytearray:
        message = self._construct_message(70, value_id, [])
        self.log.debug('Sending message [get]: %s', message)

        # send message
        async with AsyncTcpClient(ip=self.ip, port=self.port) as client:
            await client.write(message)

            n_receive_bytes = 12

            # receive response asynchronously with timeout
            data = await client.read(n_receive_bytes, timeout_s)

        values = struct.unpack('B' * n_receive_bytes, data)[6:]
        self.log.debug('Received message [get]: %s', values)

        self.cooldown_time = self.event_loop.time() + self.cooldown

        return bytearray(values)

    async def _set_value(self, value_id: int, values: list[int] | int) -> None:
        if isinstance(values, int):
            values = [values]

        message = self._construct_message(38, value_id, values)
        self.log.debug('Sending message [set]: %s', message)
        # send message
        async with AsyncTcpClient(ip=self.ip, port=self.port) as client:
            await client.write(message)

        self.cooldown_time = asyncio.get_event_loop().time() + self.cooldown

    async def get_fps(self) -> int:
        return (await self._get_value(178))[0]

    async def set_fps(self, fps: int) -> None:
        await self._set_value(178, fps)

    async def get_stream_compression(self) -> int:
        return (await self._get_value(177))[0]

    async def set_stream_compression(self, level: int) -> None:
        if not 1 <= level <= 4:
            raise ValueError('Compression level must be between 1 and 4')

        await self._set_value(177, level)

    async def get_stream_resolution(self) -> tuple[int, int]:
        wmsb, wlsb, hmsb, hlsb = (await self._get_value(179))[:4]
        return (wmsb << 8) + wlsb, (hmsb << 8) + hlsb

    async def set_stream_resolution(self, width: int, height: int) -> None:
        wmsb = width >> 8
        wlsb = width & 0xFF
        hmsb = height >> 8
        hlsb = height & 0xFF
        await self._set_value(179, [wmsb, wlsb, hmsb, hlsb])

    async def get_stream_port(self) -> int:
        pmsb, plsb = (await self._get_value(200))[:2]
        return (pmsb << 8) + plsb

    async def set_stream_port(self, port: int) -> None:
        pmsb = port >> 8
        plsb = port & 0xFF
        await self._set_value(200, [pmsb, plsb])
