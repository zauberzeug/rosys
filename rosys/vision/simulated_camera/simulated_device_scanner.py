class SimulatedDeviceScanner:

    def __init__(self, n_devices: int) -> None:
        self.n_devices = n_devices

    async def scan_for_cameras(self) -> list[str]:
        return [f'cam{i}' for i in range(self.n_devices)]

    def add_device(self) -> None:
        self.n_devices += 1

    def remove_device(self) -> None:
        self.n_devices -= min(0, self.n_devices)
