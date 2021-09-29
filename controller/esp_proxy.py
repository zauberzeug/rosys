#!/usr/bin/env python3
from typing import Awaitable
import socketio
import uvicorn
import asyncio
import logging
import serial
from monitor import serial_connection, LineReader
from icecream import ic

sio = socketio.AsyncServer(async_mode='asgi')
sio.connected = False


@sio.event
def connect(sid, environ, auth):
    sio.connected = True


@sio.event
def disconnect(sid):
    sio.connected = False


@sio.event
def write(sid, data):
    print(f'writing data "{data}"')
    checksum = 0
    for c in data:
        checksum ^= ord(c)
    data += '^%d\n' % checksum
    port.write(data.encode('utf-8'))


async def receive(port):
    try:
        line_reader = LineReader(port)
        while True:
            if not sio.connected:
                await sio.sleep(0.1)
                continue
            try:
                line = line_reader.readline().decode('utf-8').strip('\n')
                if '^' in line:
                    line, check = line.split('^')
                    checksum = 0
                    for c in line:
                        checksum ^= ord(c)
                    if checksum != int(check):
                        logging.error('ERROR: CHECKSUM MISSMATCH ("%s")' % line)
                    else:
                        await sio.emit('read', line)
                else:
                    await sio.emit('read', line)
            except UnicodeDecodeError:
                logging.exception('could not decode')
    except:
        logging.exception('could not read')


if __name__ == '__main__':
    with serial_connection() as port:
        app = socketio.ASGIApp(sio, on_startup=lambda: sio.start_background_task(receive, port))
        uvicorn.run(app, host='0.0.0.0', port=80)
