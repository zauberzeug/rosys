import rosys
import rosys.geometry
import rosys.hardware


class WheelsForCustomHardware(rosys.hardware.Wheels):

    def __init__(self) -> None:
        super().__init__()
        # TODO raise exception if hardware is not available

        # update measured velocities regularly to feed the odometer
        rosys.on_repeat(self.read_current_velocity, 0.01)

        # stop the wheels if RoSys is shutdown (or reloaded)
        rosys.on_shutdown(self.stop)

    async def drive(self, linear: float, angular: float) -> None:
        # TODO send hardware command to drive with given linear and angular velocity
        ...

    async def stop(self) -> None:
        # TODO send hardware command to stop the wheels
        ...

    async def read_current_velocity(self) -> None:
        velocities: list[rosys.geometry.Velocity] = []
        # TODO: read measured velocities from the hardware
        self.VELOCITY_MEASURED.emit(velocities)
