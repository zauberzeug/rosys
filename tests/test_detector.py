from typing import Any

import numpy as np

from rosys.vision import DetectorHardware, Image, ImageState


def _connected_detector() -> tuple[DetectorHardware, list[dict[str, Any]]]:
    """Return a detector that captures what it would emit instead of talking to a detector node."""
    detector = DetectorHardware(port=1234)
    detector.sio.connected = True
    emitted: list[dict[str, Any]] = []

    async def capture(event: str, data: dict[str, Any]) -> None:
        emitted.append({'event': event, 'data': data})
    detector.sio.emit = capture  # type: ignore[method-assign]

    return detector, emitted


def _image() -> Image:
    return Image.from_array(np.zeros((2, 2, 3), dtype=np.uint8))


async def test_upload_sends_the_requested_state():
    detector, emitted = _connected_detector()

    await detector.upload(_image(), state=ImageState.TRASH)

    assert emitted[0]['event'] == 'upload'
    assert emitted[0]['data']['metadata']['state'] == 'trash'


async def test_upload_without_state_leaves_the_choice_to_the_learning_loop():
    detector, emitted = _connected_detector()

    await detector.upload(_image())

    assert emitted[0]['data']['metadata']['state'] is None
