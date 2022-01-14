#!/usr/bin/env python3
import socketio
import uvicorn
import logging
import serial
from hardware import RobotBrain

sio = socketio.AsyncServer(async_mode='asgi')


@sio.event
def write(sid, line):
    serial.write(f'{RobotBrain.augment(line)}\n'.encode())


async def receive():
    global buffer
    try:
        while not stop_requested:
            try:
                buffer += serial.read_all().decode()
                await sio.sleep(0)
            except UnicodeDecodeError as e:
                logging.exception('could not decode', exc_info=e)
            if '\n' in buffer:
                line, buffer = buffer.split('\n', 1)
                await sio.emit('read', RobotBrain.check(line.rstrip('\r')))
    except:
        logging.exception('could not read')


if __name__ == '__main__':
    serial = serial.Serial('/dev/ttyTHS1', 115200)
    buffer = ''
    app = socketio.ASGIApp(sio, on_startup=lambda: sio.start_background_task(receive))
    stop_requested = False
    try:
        uvicorn.run(app, host='0.0.0.0', port=8081)
    except KeyboardInterrupt:
        stop_requested = True
