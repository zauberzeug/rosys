from pydantic import BaseModel


class CameraNetwork(BaseModel):

    ip: str = None
    inactive_time: float = None
    expected_throughput: float = None
    signal: float = None
    signal_average: float = None


class Camera(BaseModel):

    mac: str
    network: CameraNetwork = CameraNetwork()
