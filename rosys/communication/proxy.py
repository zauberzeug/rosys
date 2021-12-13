#!/usr/bin/env python3
import socketio
import uvicorn
import logging
from serial_communication import SerialCommunication

SerialCommunication.device_path = '/dev/ttyTHS1'
serial = SerialCommunication()

sio = socketio.AsyncServer(async_mode='asgi')
sio.connected = False


@sio.event
def connect(sid, environ, auth):
    sio.connected = True


@sio.event
def disconnect(sid):
    sio.connected = False


@sio.event
async def write(sid, line):
    await serial.send_async(line)


async def receive(port):
    try:
        while True:
            if not sio.connected:
                await sio.sleep(0.1)
                continue
            try:
                line = serial.read()
                if line is not None:
                    await sio.emit('read', line)
            except UnicodeDecodeError:
                logging.exception('could not decode')
    except:
        logging.exception('could not read')


if __name__ == '__main__':
    with serial.Serial('/dev/ttyTHS1', baudrate=115200, timeout=0.1) as port:
        app = socketio.ASGIApp(sio, on_startup=lambda: sio.start_background_task(receive, port))
        uvicorn.run(app, host='0.0.0.0', port=8081)
