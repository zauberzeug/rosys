import logging

from ... import rosys
from ...rosys import Repeater
from .camera import Camera

log = logging.getLogger('rosys.vision.reconnecting_camera')


class ReconnectingCamera(Camera):
    """Camera mixin that automatically reconnects on connection loss.

    This class can be used as a base class for cameras that should automatically
    attempt to reconnect when the connection is lost. It wraps ``connect()`` to
    catch failures and starts a background task that periodically retries.

    Example usage::

        class MyReconnectingCamera(ReconnectingCamera, MyCamera):
            pass
    """

    def __init__(self, *, reconnect_interval: float = 5.0, **kwargs) -> None:
        super().__init__(**kwargs)
        self.reconnect_interval = reconnect_interval
        self._reconnect_repeater: Repeater | None = None

    def to_dict(self) -> dict:
        return super().to_dict() | {'reconnect_interval': self.reconnect_interval}

    @property
    def is_activated(self) -> bool:
        """Whether the camera is activated (reconnect task running)."""
        return bool(self._reconnect_repeater and self._reconnect_repeater.running)

    async def connect(self) -> None:
        try:
            await super().connect()
        except Exception as e:
            log.warning('[%s] connect failed: %s', self.id, e)
        if not self._reconnect_repeater:
            self._reconnect_repeater = rosys.on_repeat(self._try_reconnect, self.reconnect_interval)
        if not self._reconnect_repeater.running:
            self._reconnect_repeater.start()
        log.debug('[%s] reconnect task started', self.id)
        self.connect_after_init = True

    async def disconnect(self) -> None:
        if self._reconnect_repeater:
            self._reconnect_repeater.stop()
            self._reconnect_repeater = None
        try:
            await super().disconnect()
        except Exception as e:
            log.warning('[%s] disconnect failed: %s', self.id, e)

    async def _try_reconnect(self) -> None:
        if not self.is_connected:
            try:
                log.debug('[%s] trying to reconnect', self.id)
                await super().connect()
            except Exception as e:
                log.error('reconnection attempt failed: %s', e)
