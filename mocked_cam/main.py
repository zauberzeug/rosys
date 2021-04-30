from fastapi import FastAPI

app = FastAPI()


if __name__ == "__main__":
    uvicorn.run(
        "main:app", host="0.0.0.0", port=8080,
        reload=True, lifespan='on', forwarded_allow_ips='*', proxy_headers=True
    )