from fastapi import FastAPI
from fastapi_socketio import SocketManager

app = FastAPI()
sio = SocketManager(app=app)


@sio.on('drive_power')
def drive_power(sid, data):
    print(f'{sid} sends {data}', flush=True)


@app.on_event("startup")
async def startup():

    print("---- startup", flush=True)


@app.on_event("shutdown")
async def shutdown():

    print("---- stop", flush=True)


@app.get("/api")
def main():
    return {"status": "hello, I'm the robot system!"}
