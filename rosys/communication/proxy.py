#!/usr/bin/env python3
import socketio
import uvicorn
import logging
import aioserial
from checksum import augment, check

aioserial = aioserial.AioSerial('/dev/ttyTHS1', baudrate=115200)
buffer = ''

sio = socketio.AsyncServer(async_mode='asgi')
sio.connected = False

stop_requested = False


@sio.event
def connect(sid, environ, auth):
    sio.connected = True


@sio.event
def disconnect(sid):
    sio.connected = False


@sio.event
async def write(sid, line):
     await aioserial.write_async(f'{augment(line)}\n'.encode())


async def receive():
    global buffer
    try:
        while not stop_requested:
            if not sio.connected:
                await sio.sleep(0.1)
                continue
            try:
                buffer += aioserial.read_all().decode()
                await sio.sleep(0)
            except UnicodeDecodeError:
                logging.exception('could not decode')
            if '\n' not in buffer:
                continue
            line, buffer = buffer.split('\n', 1)
            await sio.emit('read', line)
    except:
        logging.exception('could not read')


if __name__ == '__main__':
    try:
        app = socketio.ASGIApp(sio, on_startup=lambda: sio.start_background_task(receive))
        uvicorn.run(app, host='0.0.0.0', port=8081)
    except KeyboardInterrupt:
        stop_requested = True
