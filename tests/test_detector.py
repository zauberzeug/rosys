from datetime import datetime
from typing import Any

import numpy as np
import pytest

from rosys.vision import DetectorHardware, Image

Payloads = dict[str, dict[str, Any]]


@pytest.fixture
def payloads() -> Payloads:
    return {}


@pytest.fixture
def detector(monkeypatch: pytest.MonkeyPatch, payloads: Payloads) -> DetectorHardware:
    async def capture(event: str, data: dict[str, Any], **_: Any) -> dict[str, Any]:
        payloads[event] = data
        return {'items': [{} for _ in data.get('images', [])]}

    detector = DetectorHardware(port=1234)
    monkeypatch.setattr(detector.sio, 'connected', True)
    monkeypatch.setattr(detector.sio, 'emit', capture)
    monkeypatch.setattr(detector.sio, 'call', capture)
    return detector


@pytest.mark.parametrize('creation_date, expected', [
    (datetime(2020, 1, 1, 12, 0, 0), {'created': '2020-01-01T12:00:00'}),
    ('2020-01-01T12:00:00', {'created': '2020-01-01T12:00:00'}),
    (None, {}),  # an absent key lets the node fall back to its own time
])
async def test_upload_sends_the_creation_date_as_created(detector: DetectorHardware, payloads: Payloads,
                                                         creation_date: datetime | str | None, expected: dict):
    await detector.upload(Image.from_array(np.zeros((2, 2, 3), dtype=np.uint8)), creation_date=creation_date)
    assert payloads['upload']['metadata'] == {'source': None, 'tags': [], **expected}


async def test_detect_keeps_the_creation_date_key(detector: DetectorHardware, payloads: Payloads):
    image = Image.from_array(np.zeros((2, 2, 3), dtype=np.uint8))
    await detector.detect(image, lazy=False, creation_date='2020-01-01T12:00:00')
    await detector.batch_detect([image], creation_date='2020-01-01T12:00:00')
    assert payloads['detect']['creation_date'] == payloads['batch_detect']['creation_date'] == '2020-01-01T12:00:00'
