from datetime import datetime
from typing import Any

import numpy as np
import pytest

from rosys.geometry import Point
from rosys.vision import DetectorHardware, Image, ImageState
from rosys.vision.detections import (
    BoxDetection,
    ClassificationDetection,
    Detections,
    PointDetection,
    SegmentationDetection,
    Shape,
)

Payloads = dict[str, dict[str, Any]]


@pytest.fixture
def payloads() -> Payloads:
    return {}


@pytest.fixture
def answer() -> dict[str, Any]:
    """The detections the detector node answers for every image; none unless a test adds some."""
    return {}


@pytest.fixture
def detector(monkeypatch: pytest.MonkeyPatch, payloads: Payloads, answer: dict[str, Any]) -> DetectorHardware:
    async def capture(event: str, data: dict[str, Any], **_: Any) -> dict[str, Any]:
        payloads[event] = data
        return {'items': [answer for _ in data['images']]} if event == 'batch_detect' else answer

    detector = DetectorHardware(port=1234)
    monkeypatch.setattr(detector.sio, 'connected', True)
    monkeypatch.setattr(detector.sio, 'emit', capture)
    monkeypatch.setattr(detector.sio, 'call', capture)
    return detector


def _image() -> Image:
    return Image.from_array(np.zeros((2, 2, 3), dtype=np.uint8))


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


async def test_detections_are_read_from_the_answer_of_the_node(detector: DetectorHardware, answer: dict[str, Any]):
    """Every kind of detection the node answers with arrives, its pixel coordinates as floats and the keys
    rosys has no field for, such as the category id, left out."""
    answer.update({
        'box_detections': [{'category_name': 'weed', 'x': 10, 'y': 20, 'width': 30, 'height': 40,
                            'model_name': '1.2', 'confidence': 0.9, 'category_id': 'c1'}],
        'point_detections': [{'category_name': 'crop', 'x': 1.5, 'y': 2.5,
                              'model_name': '1.2', 'confidence': 0.8, 'category_id': 'c2'}],
        'segmentation_detections': [{'category_name': 'row', 'shape': {'points': [{'x': 0, 'y': 0}, {'x': 4, 'y': 3}]},
                                     'model_name': '1.2', 'confidence': 0.7, 'category_id': 'c3'}],
        'classification_detections': [{'category_name': 'field', 'model_name': '1.2', 'confidence': 0.6,
                                       'category_id': 'c4'}],
    })
    image = _image()

    detections = await detector.detect(image, lazy=False)
    batch_detections = await detector.batch_detect([image])

    assert detections == Detections(
        boxes=[BoxDetection(category_name='weed', model_name='1.2', confidence=0.9, x=10, y=20, width=30, height=40)],
        points=[PointDetection(category_name='crop', model_name='1.2', confidence=0.8, x=1.5, y=2.5)],
        segmentations=[SegmentationDetection(category_name='row', model_name='1.2', confidence=0.7,
                                             shape=Shape(points=[Point(x=0, y=0), Point(x=4, y=3)]))],
        classifications=[ClassificationDetection(category_name='field', model_name='1.2', confidence=0.6)],
    )
    assert batch_detections == [detections]
    assert detections is not None
    box, segmentation = detections.boxes[0], detections.segmentations[0]
    coordinates = [box.x, box.y, box.width, box.height, *(c for p in segmentation.shape.points for c in p.tuple)]
    assert all(isinstance(value, float) for value in coordinates)
