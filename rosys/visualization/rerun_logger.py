from __future__ import annotations

import logging
from pathlib import Path
from typing import TYPE_CHECKING, ClassVar

import rerun as rr

from .. import rosys
from ..geometry import Prism

if TYPE_CHECKING:
    from ..driving import Odometer
    from ..vision import CameraProvider, Image

log = logging.getLogger('rosys.visualization.rerun_logger')


class RerunLogger:
    """Logs rosys data to Rerun for visualization.

    This class wraps the Rerun SDK to log camera images, robot poses, and other data
    from rosys components. It starts a gRPC server for data and an HTTP server for
    the web viewer that clients can connect to for live visualization.

    :param app_id: Application identifier for Rerun (default: 'rosys')
    :param grpc_port: gRPC port for data streaming (default: 9876)
    :param web_port: HTTP port for the web viewer (default: 9877)
    """

    _instances: ClassVar[dict[tuple[int, int], RerunLogger]] = {}
    """Track existing instances by (grpc_port, web_port) to prevent duplicate server creation."""

    def __new__(cls, *, app_id: str = 'rosys', grpc_port: int = 9876, web_port: int = 9877) -> RerunLogger:
        key = (grpc_port, web_port)
        if key in cls._instances:
            log.info('Reusing existing RerunLogger for ports %d/%d', grpc_port, web_port)
            return cls._instances[key]
        instance = super().__new__(cls)
        cls._instances[key] = instance
        return instance

    def __init__(self, *, app_id: str = 'rosys', grpc_port: int = 9876, web_port: int = 9877) -> None:
        if hasattr(self, '_initialized'):
            return
        self._initialized = True

        self.app_id = app_id
        self.grpc_port = grpc_port
        self.web_port = web_port
        self._recording_path: Path | None = None
        self._connected_camera_providers: list[CameraProvider] = []
        self._connected_odometers: list[Odometer] = []

        rr.init(app_id)
        try:
            grpc_uri = rr.serve_grpc(grpc_port=grpc_port)
            self.web_viewer_url = rr.serve_web_viewer(web_port=web_port, connect_to=grpc_uri, open_browser=False)
            log.info('Started Rerun gRPC server on port %d, web viewer on port %d', grpc_port, web_port)
        except RuntimeError as e:
            if 'Address already in use' in str(e):
                log.warning('Rerun server ports already in use. Connecting to existing instance.')
                self.web_viewer_url = f'http://0.0.0.0:{web_port}'
            else:
                raise

    def connect_camera_provider(self, provider: CameraProvider) -> None:
        """Connect a camera provider to log images to Rerun.

        Subscribes to the provider's NEW_IMAGE event and logs each image
        to the Rerun timeline under the entity path `camera/{camera_id}`.

        :param provider: The camera provider to connect
        """
        if provider in self._connected_camera_providers:
            log.warning('Camera provider already connected')
            return

        provider.NEW_IMAGE.subscribe(self._log_image)
        self._connected_camera_providers.append(provider)
        log.info('Connected camera provider to Rerun logger')

    def disconnect_camera_provider(self, provider: CameraProvider) -> None:
        """Disconnect a camera provider from Rerun logging.

        :param provider: The camera provider to disconnect
        """
        if provider not in self._connected_camera_providers:
            log.warning('Camera provider not connected')
            return

        provider.NEW_IMAGE.unsubscribe(self._log_image)
        self._connected_camera_providers.remove(provider)
        log.info('Disconnected camera provider from Rerun logger')

    def connect_odometer(self, odometer: Odometer) -> None:
        """Connect an odometer to log robot poses to Rerun.

        Subscribes to the odometer's PREDICTION_UPDATED event and logs
        the robot pose as a Transform3D under the entity path `robot/pose`.

        :param odometer: The odometer to connect
        """
        if odometer in self._connected_odometers:
            log.warning('Odometer already connected')
            return

        odometer.PREDICTION_UPDATED.subscribe(lambda: self._log_pose(odometer))
        self._connected_odometers.append(odometer)
        log.info('Connected odometer to Rerun logger')

    def start_recording(self, path: Path) -> None:
        """Start recording data to an RRD file.

        The recording can be stopped with `stop_recording()` and the file
        can later be loaded in the Rerun viewer for replay.

        :param path: Path to save the RRD file
        """
        path = path.expanduser()
        path.parent.mkdir(parents=True, exist_ok=True)
        self._recording_path = path
        rr.save(str(path))
        log.info('Started recording to %s', path)

    def stop_recording(self) -> Path | None:
        """Stop recording and return the path to the RRD file.

        :return: Path to the recorded RRD file, or None if not recording
        """
        if self._recording_path is None:
            log.warning('Not currently recording')
            return None

        path = self._recording_path
        self._recording_path = None
        log.info('Stopped recording to %s', path)
        return path

    @property
    def is_recording(self) -> bool:
        """Whether recording to an RRD file is currently active."""
        return self._recording_path is not None

    def _log_image(self, image: Image) -> None:
        """Log an image to Rerun."""
        rr.set_time('rosys_time', duration=image.time)
        rr.log(f'camera/{image.camera_id}', rr.Image(image.array))

    def _log_pose(self, odometer: Odometer) -> None:
        """Log the robot pose to Rerun."""
        pose = odometer.prediction
        rr.set_time('rosys_time', duration=rosys.time())
        rr.log(
            'robot/pose',
            rr.Transform3D(
                translation=[pose.x, pose.y, 0.0],
                rotation=rr.RotationAxisAngle(axis=[0, 0, 1], radians=pose.yaw),
            ),
        )
        # Log robot shape as wireframe
        shape = Prism.default_robot_shape()
        bottom = [[x, y, 0.0] for x, y in shape.outline]
        bottom.append(bottom[0])  # close the loop
        top = [[x, y, shape.height] for x, y, _ in bottom]
        verticals = [[[x, y, 0.0], [x, y, shape.height]] for x, y in shape.outline]
        rr.log('robot/pose/shape', rr.LineStrips3D([bottom, top, *verticals]))

    def log_custom(self, entity_path: str, *args, **kwargs) -> None:
        """Log custom data to Rerun.

        This is a pass-through to `rr.log()` for advanced use cases
        where you need to log custom archetypes.

        :param entity_path: The entity path to log to
        :param args: Positional arguments passed to rr.log()
        :param kwargs: Keyword arguments passed to rr.log()
        """
        rr.log(entity_path, *args, **kwargs)
