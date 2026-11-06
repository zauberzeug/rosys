import asyncio
from collections.abc import Callable
from datetime import datetime
from typing import Any

import numpy as np
import pytest

from rosys.vision import (
    BatchImageMetadata,
    Detector,
    DetectorHardware,
    DetectorSimulation,
    Image,
    ImageState,
    ImageTransmissionFormat,
    SimulatedCameraProvider,
)
from rosys.vision.image_processing import decode_jpeg_image

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


def _image(camera_id: str = 'cam') -> Image:
    return Image.from_array(np.zeros((2, 2, 3), dtype=np.uint8), camera_id=camera_id)


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
    assert payloads['detect']['image'] == payloads['upload']['image'] == expected
    assert payloads['batch_detect']['images'][0] == \
        {**expected, 'camera_id': 'cam', 'tags': [], 'source': None, 'creation_date': None}


async def test_jpeg_transmission_encodes_the_images_before_sending(make_detector: Callable[..., DetectorHardware],
                                                                   payloads: Payloads):
    detector = make_detector(image_transmission_format=ImageTransmissionFormat.JPEG, jpeg_quality=95)
    image = _image()
    await detector.detect(image, lazy=False)
    await detector.batch_detect([image])
    await detector.upload(image)
    batch_image = payloads['batch_detect']['images'][0]
    assert payloads['detect']['image'].keys() == payloads['upload']['image'].keys() == {'format', 'bytes'}
    assert batch_image.keys() == {'format', 'bytes', 'camera_id', 'tags', 'source', 'creation_date'}
    for sent in (payloads['detect']['image'], batch_image, payloads['upload']['image']):
        assert sent['format'] == 'jpeg'
        decoded = decode_jpeg_image(sent['bytes'])
        assert decoded is not None
        assert decoded.shape == (2, 2, 3)


async def test_batch_detect_sends_the_camera_of_each_image(detector: DetectorHardware, payloads: Payloads):
    await detector.batch_detect([_image('front'), _image('back')])
    assert [sent['camera_id'] for sent in payloads['batch_detect']['images']] == ['front', 'back']


async def test_batch_detect_sends_the_metadata_of_each_image(detector: DetectorHardware, payloads: Payloads):
    await detector.batch_detect([_image(), _image()], tags=['robot'], source='robot', image_metadata=[
        BatchImageMetadata(tags=['front'], source='robot/front', creation_date=datetime(2020, 1, 1, 12, 0, 0)),
        BatchImageMetadata(),
    ])
    first, second = payloads['batch_detect']['images']
    assert (first['tags'], first['source'], first['creation_date']) == \
        (['front'], 'robot/front', '2020-01-01T12:00:00')
    assert (second['tags'], second['source'], second['creation_date']) == ([], None, None)
    assert (payloads['batch_detect']['tags'], payloads['batch_detect']['source']) == (['robot'], 'robot')


@pytest.mark.parametrize('kind', ['hardware', 'simulation'])
async def test_batch_detect_rejects_image_metadata_that_does_not_match_the_images(
        make_detector: Callable[..., DetectorHardware], kind: str):
    detector: Detector = make_detector() if kind == 'hardware' else DetectorSimulation(SimulatedCameraProvider())
    with pytest.raises(ValueError):
        # without the check the simulation waits for a rosys time that nothing advances
        await asyncio.wait_for(detector.batch_detect([_image(), _image()], image_metadata=[BatchImageMetadata()]), 1)


def test_an_invalid_jpeg_quality_is_rejected_when_the_detector_is_created():
    with pytest.raises(ValueError):
        DetectorHardware(port=1234, jpeg_quality=0)
