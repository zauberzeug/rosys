#!/usr/bin/env python3
import socketio
import uvicorn
import logging
import serial
from .checksum import augment, check


class LineReader:
    # https://github.com/pyserial/pyserial/issues/216#issuecomment-369414522

    def __init__(self, s: serial.Serial):
        self.buf = bytearray()
        self.s = s

    def readline(self):
        i = self.buf.find(b"\n")
        if i >= 0:
            r = self.buf[:i+1]
            self.buf = self.buf[i+1:]
            return r
        while True:
            i = max(1, min(2048, self.s.in_waiting))
            data = self.s.read(i)
            i = data.find(b"\n")
            if i >= 0:
                r = self.buf + data[:i+1]
                self.buf[0:] = data[i+1:]
                return r
            else:
                self.buf.extend(data)


sio = socketio.AsyncServer(async_mode='asgi')
sio.connected = False


@sio.event
def connect(sid, environ, auth):
    sio.connected = True


@sio.event
def disconnect(sid):
    sio.connected = False


@sio.event
def write(sid, line):
    port.write((f'{augment(line)}\n').encode('utf-8'))


async def receive(port):
    try:
        line_reader = LineReader(port)
        while True:
            if not sio.connected:
                await sio.sleep(0.1)
                continue
            try:
                line = check(line_reader.readline().decode('utf-8').strip('\r\n'))
                if line is not None:
                    await sio.emit('read', line)
            except UnicodeDecodeError:
                logging.exception('could not decode')
    except:
        logging.exception('could not read')


if __name__ == '__main__':
    with serial.Serial('/dev/ttyTHS1', baudrate=115200, timeout=0.1) as port:
        app = socketio.ASGIApp(sio, on_startup=lambda: sio.start_background_task(receive, port))
        uvicorn.run(app, host='0.0.0.0', port=80)
