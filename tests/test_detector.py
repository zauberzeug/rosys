import asyncio
from collections.abc import Callable
from datetime import datetime
from typing import Any

import numpy as np
import pytest

from rosys import run
from rosys.geometry import Point, Point3d, Polygon
from rosys.testing import forward
from rosys.vision import (
    CalibratableCamera,
    DetectorHardware,
    DetectorSimulation,
    Image,
    ImageMasks,
    ImageState,
    ImageTransmissionFormat,
    SimulatedObject,
)
from rosys.vision.detector import DetectorException
from rosys.vision.image_processing import decode_jpeg_image, encode_image_as_jpeg

Payloads = dict[str, dict[str, Any]]


@pytest.fixture
def payloads() -> Payloads:
    return {}


@pytest.fixture
def make_detector(monkeypatch: pytest.MonkeyPatch, payloads: Payloads) -> Callable[..., DetectorHardware]:
    """Build a detector whose socket.io messages are captured in ``payloads`` instead of being sent."""
    async def capture(event: str, data: dict[str, Any], **_: Any) -> dict[str, Any]:
        payloads[event] = data
        return {'items': [{} for _ in data.get('images', [])]}

    def make(**kwargs: Any) -> DetectorHardware:
        detector = DetectorHardware(port=1234, **kwargs)
        monkeypatch.setattr(detector.sio, 'connected', True)
        monkeypatch.setattr(detector.sio, 'emit', capture)
        monkeypatch.setattr(detector.sio, 'call', capture)
        return detector
    return make


@pytest.fixture
def detector(make_detector: Callable[..., DetectorHardware]) -> DetectorHardware:
    return make_detector()


def _image() -> Image:
    return Image.from_array(np.zeros((2, 2, 3), dtype=np.uint8))


LEFT_HALF = Polygon(outline=[Point(x=0, y=0), Point(x=0.5, y=0), Point(x=0.5, y=1), Point(x=0, y=1)])


def _camera_image(camera_id: str = 'cam') -> Image:
    return Image.from_array(np.random.randint(1, 255, (4, 8, 3), dtype=np.uint8), camera_id=camera_id)


@pytest.mark.parametrize('creation_date, expected', [
    (datetime(2020, 1, 1, 12, 0, 0), {'created': '2020-01-01T12:00:00'}),
    ('2020-01-01T12:00:00', {'created': '2020-01-01T12:00:00'}),
    (None, {}),  # an absent key lets the node fall back to its own time
])
async def test_upload_sends_the_creation_date_as_created(detector: DetectorHardware, payloads: Payloads,
                                                         creation_date: datetime | str | None, expected: dict):
    await detector.upload(_image(), creation_date=creation_date)
    assert payloads['upload']['metadata'] == {'source': None, 'tags': [], 'state': None, **expected}


async def test_detect_keeps_the_creation_date_key(detector: DetectorHardware, payloads: Payloads):
    image = _image()
    await detector.detect(image, lazy=False, creation_date='2020-01-01T12:00:00')
    await detector.batch_detect([image], creation_date='2020-01-01T12:00:00')
    assert payloads['detect']['creation_date'] == payloads['batch_detect']['creation_date'] == '2020-01-01T12:00:00'


async def test_upload_sends_the_requested_state(detector: DetectorHardware, payloads: Payloads):
    await detector.upload(_image(), state=ImageState.TRASH)
    assert payloads['upload']['metadata']['state'] == 'trash'


async def test_upload_without_state_leaves_the_choice_to_the_learning_loop(detector: DetectorHardware,
                                                                           payloads: Payloads):
    await detector.upload(_image())
    assert payloads['upload']['metadata']['state'] is None


async def test_images_are_sent_as_raw_pixel_arrays_by_default(detector: DetectorHardware, payloads: Payloads):
    image = _image()
    await detector.detect(image, lazy=False)
    await detector.batch_detect([image])
    await detector.upload(image)
    expected = {'bytes': image.array.tobytes(), 'dtype': 'uint8', 'shape': (2, 2, 3)}
    assert payloads['detect']['image'] == payloads['batch_detect']['images'][0] == payloads['upload']['image'] == expected


async def test_jpeg_transmission_encodes_the_images_before_sending(make_detector: Callable[..., DetectorHardware],
                                                                   payloads: Payloads):
    detector = make_detector(image_transmission_format=ImageTransmissionFormat.JPEG, jpeg_quality=95)
    image = _image()
    await detector.detect(image, lazy=False)
    await detector.batch_detect([image])
    await detector.upload(image)
    for sent in (payloads['detect']['image'], payloads['batch_detect']['images'][0], payloads['upload']['image']):
        assert sent.keys() == {'format', 'bytes'}
        assert sent['format'] == 'jpeg'
        decoded = decode_jpeg_image(sent['bytes'])
        assert decoded is not None
        assert decoded.shape == (2, 2, 3)


def test_an_invalid_jpeg_quality_is_rejected_when_the_detector_is_created():
    with pytest.raises(ValueError):
        DetectorHardware(port=1234, jpeg_quality=0)


async def test_masked_cameras_send_the_masked_pixels_and_keep_the_image_untouched(
        make_detector: Callable[..., DetectorHardware], payloads: Payloads):
    detector = make_detector(masks={'cam': [LEFT_HALF]})
    image = _camera_image()
    original = image.array.copy()
    await detector.detect(image, lazy=False)
    await detector.batch_detect([image])
    await detector.upload(image)
    expected = detector.masks.apply(image)
    assert np.all(expected[:, :5] == ImageMasks.FILL_VALUE)
    for sent in (payloads['detect']['image'], payloads['batch_detect']['images'][0], payloads['upload']['image']):
        assert sent == {'bytes': expected.tobytes(), 'dtype': 'uint8', 'shape': (4, 8, 3)}
    assert np.array_equal(image.array, original)


async def test_masked_cameras_are_encoded_as_jpeg_after_masking(make_detector: Callable[..., DetectorHardware],
                                                                payloads: Payloads):
    detector = make_detector(image_transmission_format=ImageTransmissionFormat.JPEG, masks={'cam': [LEFT_HALF]})
    image = _camera_image()
    await detector.detect(image, lazy=False)
    await detector.batch_detect([image])
    await detector.upload(image)
    expected = encode_image_as_jpeg(detector.masks.apply(image), detector.jpeg_quality)
    for sent in (payloads['detect']['image'], payloads['batch_detect']['images'][0], payloads['upload']['image']):
        assert sent == {'format': 'jpeg', 'bytes': expected}


async def test_cameras_without_masks_are_sent_unchanged(make_detector: Callable[..., DetectorHardware],
                                                        payloads: Payloads):
    detector = make_detector(masks={'cam': [LEFT_HALF]})
    image = _camera_image('other')
    await detector.detect(image, lazy=False)
    assert payloads['detect']['image']['bytes'] == image.array.tobytes()


async def test_masks_can_be_set_after_creation(make_detector: Callable[..., DetectorHardware], payloads: Payloads):
    detector = make_detector()
    detector.masks.set('cam', [LEFT_HALF])
    image = _camera_image()
    await detector.detect(image, lazy=False)
    assert payloads['detect']['image']['bytes'] == detector.masks.apply(image).tobytes()


async def test_interrupted_masking_cancels_the_call_instead_of_failing(make_detector: Callable[..., DetectorHardware],
                                                                       monkeypatch: pytest.MonkeyPatch):
    async def interrupted(*_: Any, **__: Any) -> None:
        return None
    monkeypatch.setattr(run, 'io_bound', interrupted)
    detector = make_detector(masks={'cam': [LEFT_HALF]})
    with pytest.raises(asyncio.CancelledError):
        await detector.upload(_camera_image())
    with pytest.raises(DetectorException, match='Detection cancelled'):
        await detector.detect(_camera_image(), lazy=False)


class _Cameras:

    def __init__(self, camera: CalibratableCamera) -> None:
        self.cameras = {camera.id: camera}


async def test_the_simulation_does_not_detect_objects_inside_the_masks(rosys_integration):
    camera = CalibratableCamera.create_calibrated(id='cam', width=800, height=600, connect_after_init=False)
    detector = DetectorSimulation(_Cameras(camera), noise=0, masks={'cam': [LEFT_HALF]})
    detector.simulated_objects = [SimulatedObject(category_name='obj', position=Point3d(x=x, y=0, z=0))
                                  for x in (-0.3, 0.3)]
    image = Image.from_array(np.zeros((600, 800, 3), dtype=np.uint8), camera_id='cam')

    task = asyncio.create_task(detector.detect(image))
    await forward(1.0)
    detections = await task

    assert detections is not None
    assert len(detections.points) == 1
    assert detections.points[0].x > 400
