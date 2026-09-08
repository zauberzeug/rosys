from datetime import datetime
from typing import Any

import numpy as np

from rosys.vision import DetectorHardware, Image


async def test_upload_keys_the_creation_date_as_created():
    """The upload metadata is parsed against ``ImageMetadata``, which declares ``created``.

    The ``detect`` payloads keep ``creation_date``: the detector node reads that one itself.
    """
    detector = DetectorHardware(port=1234)
    detector.sio.connected = True
    emitted: list[dict[str, Any]] = []

    async def capture(event: str, data: dict[str, Any]) -> None:
        emitted.append(data)
    detector.sio.emit = capture  # type: ignore[method-assign]

    await detector.upload(Image.from_array(np.zeros((2, 2, 3), dtype=np.uint8)),
                          creation_date=datetime(2020, 1, 1, 12, 0, 0))

    assert emitted[0]['metadata']['created'] == '2020-01-01T12:00:00'
