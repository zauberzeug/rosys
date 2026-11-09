import asyncio
import weakref
from collections import deque
from types import SimpleNamespace
from typing import Any

import numpy as np
import pytest

from rosys.vision import image_route
from rosys.vision.image import Image
from rosys.vision.image_route import _try_get_jpeg

MAX_CONCURRENT_ENCODINGS = 3
"""The bound in these tests, independent of the CPUs of the machine running them."""


class _Encoder:
    """Stands in for ``run.cpu_bound``: records what it is asked to encode and holds every encoding until released."""

    def __init__(self) -> None:
        self.running = 0
        self.most_running = 0
        self.arrays: list[np.ndarray] = []
        self.release = asyncio.Event()

    async def __call__(self, _callback: Any, array: np.ndarray, *_args: Any) -> bytes:
        self.running += 1
        self.most_running = max(self.most_running, self.running)
        self.arrays.append(array)
        try:
            await self.release.wait()
            return f'jpeg {array.shape}'.encode()
        finally:
            self.running -= 1


@pytest.fixture
def encoder(monkeypatch: pytest.MonkeyPatch) -> _Encoder:
    encoder = _Encoder()
    monkeypatch.setattr(image_route.run, 'cpu_bound', encoder)
    monkeypatch.setattr(image_route, 'MAX_CONCURRENT_ENCODINGS', MAX_CONCURRENT_ENCODINGS)  # read per event loop
    monkeypatch.setattr(image_route, '_encodings', weakref.WeakKeyDictionary())  # tests may share one event loop
    return encoder


def _camera(times: range, *, height: int = 6, width: int = 9) -> Any:
    images = [Image.from_array(np.zeros((height, width, 3), dtype=np.uint8), camera_id='cam', time=float(t + 1))  # NOTE: from_array replaces a time of 0
              for t in times]
    return SimpleNamespace(id='cam', images=deque(images), calibration=None)


async def _request(camera: Any, time: int, *, shrink: float = 1.0) -> bytes | None:
    return await _try_get_jpeg(camera, str(float(time + 1)), shrink=shrink, max_dimension=None,
                               undistort=False, fast=True, compression=90)


async def _settle() -> None:
    for _ in range(10):
        await asyncio.sleep(0)


async def test_a_burst_of_requests_encodes_only_a_few_frames_at_a_time(encoder: _Encoder) -> None:
    camera = _camera(range(20))
    requests = [asyncio.create_task(_request(camera, t)) for t in range(20)]
    await _settle()
    assert encoder.running == MAX_CONCURRENT_ENCODINGS

    encoder.release.set()
    results = await asyncio.gather(*requests)

    assert encoder.most_running == MAX_CONCURRENT_ENCODINGS
    assert all(result is not None for result in results)


async def test_a_frame_that_left_the_buffer_while_waiting_is_not_encoded(encoder: _Encoder) -> None:
    """Waiting requests hold no frame, so frames the camera dropped meanwhile cost nothing."""
    camera = _camera(range(2 * MAX_CONCURRENT_ENCODINGS))
    requests = [asyncio.create_task(_request(camera, t)) for t in range(2 * MAX_CONCURRENT_ENCODINGS)]
    await _settle()
    camera.images.clear()

    encoder.release.set()
    results = await asyncio.gather(*requests)

    assert len(encoder.arrays) == MAX_CONCURRENT_ENCODINGS
    assert results[MAX_CONCURRENT_ENCODINGS:] == [None] * MAX_CONCURRENT_ENCODINGS


async def test_clients_asking_for_the_same_image_share_one_encoding(encoder: _Encoder) -> None:
    camera = _camera(range(1))
    requests = [asyncio.create_task(_request(camera, 0)) for _ in range(3)]
    await _settle()
    encoder.release.set()
    results = await asyncio.gather(*requests)
    later = await _request(camera, 0)

    assert len(encoder.arrays) == 1
    assert results == [later] * 3
    assert await _request(camera, 0, shrink=3) != later  # other parameters are another image
    assert len(encoder.arrays) == 2


async def test_a_fast_shrink_sends_only_the_kept_pixels_to_the_worker(encoder: _Encoder) -> None:
    encoder.release.set()
    camera = _camera(range(1), height=6, width=9)

    await _request(camera, 0, shrink=3)

    assert encoder.arrays[0].shape == (2, 3, 3)
    assert np.shares_memory(encoder.arrays[0], camera.images[0].array)  # a view; pickling copies only its pixels
