from fastapi import FastAPI
from fastapi_socketio import SocketManager

app = FastAPI()
sio = SocketManager(app=app)


@app.on_event("startup")
async def startup():

    print("---- startup")


@app.on_event("shutdown")
async def shutdown():

    print("---- stop")


@app.get("/api")
def main():
    return {"status": "hello, I'm the robot system!"}
