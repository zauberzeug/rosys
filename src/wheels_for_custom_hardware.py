import rosys
import rosys.hardware


class WheelsForCustomHardware(rosys.hardware.Wheels):

    def __init__(self) -> None:
        super().__init__()
        # TODO raise exception if hardware is not available

        # measured velocities must be updated regularly to feed the odometer
        rosys.on_repeat(self.read_current_velocity, 0.01)
        # this makes the wheels stop if rosys is shutdown (or reloaded)
        rosys.on_shutdown(self.stop)

    async def drive(self, linear: float, angular: float) -> None:
        # TODO send driving commands for given linear and angular to the hardware
        ...

    async def stop(self) -> None:
        # TODO perform hardware command to stop the wheels
        ...

    async def read_current_velocity(self) -> None:
        velocities: list[rosys.Velocity] = []
        # TODO: read measured velocities since last call from hardware
        self.VELOCITY_MEASURED.emit(velocities)
