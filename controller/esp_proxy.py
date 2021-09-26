#!/usr/bin/env python3
import socketio
import uvicorn

sio = socketio.AsyncServer(async_mode='asgi')


@sio.event
def connect(sid, environ, auth):
    print('connect ', sid)


@sio.event
def disconnect(sid):
    print('disconnect ', sid)


@sio.event
def write(sid, data):
    print(f'writing "{data}" to serial')
    pass


async def read():
    count = 0
    while True:
        await sio.emit('read', f'step #{count}')
        print(f'count {count}')
        count += 1
        await sio.sleep(5)

app = socketio.ASGIApp(sio, on_startup=lambda: sio.start_background_task(read))

if __name__ == '__main__':
    uvicorn.run(app, host='0.0.0.0', port=80)
