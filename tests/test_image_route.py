import asyncio
import pickle
from collections import deque
from types import SimpleNamespace
from typing import Any

import numpy as np
import pytest

from rosys.vision import image_route
from rosys.vision.image import Image
from rosys.vision.image_rotation import ImageRotation
from rosys.vision.image_route import _process, _try_get_jpeg

MAX_CONCURRENT_ENCODINGS = 3
"""The bound in these tests, independent of the CPUs of the machine running them."""


class _Encoder:
    """Stands in for ``run.cpu_bound``: records what it is asked to encode and holds every encoding until released."""

    def __init__(self) -> None:
        self.running = 0
        self.most_running = 0
        self.arrays: list[np.ndarray] = []
        self.release = asyncio.Event()
        self.result: bytes | None = b'jpeg'

    async def __call__(self, _callback: Any, array: np.ndarray, *_args: Any) -> bytes | None:
        self.running += 1
        self.most_running = max(self.most_running, self.running)
        self.arrays.append(array)
        try:
            await self.release.wait()
            return None if self.result is None else self.result + f' {array.shape}'.encode()
        finally:
            self.running -= 1


@pytest.fixture
def encoder(monkeypatch: pytest.MonkeyPatch) -> _Encoder:
    encoder = _Encoder()
    monkeypatch.setattr(image_route.run, 'cpu_bound', encoder)
    monkeypatch.setattr(image_route, 'MAX_CONCURRENT_ENCODINGS', MAX_CONCURRENT_ENCODINGS)
    monkeypatch.setattr(image_route, '_encoding', image_route._Encoding())  # NOTE: with the bound of these tests
    return encoder


def _camera(times: range, *, height: int = 6, width: int = 9, array: np.ndarray | None = None) -> Any:
    # NOTE: from_array replaces a time of 0 with the current time, so the frames start at 1
    images = [Image.from_array(np.zeros((height, width, 3), dtype=np.uint8) if array is None else array,
                               camera_id='cam', time=float(t + 1))
              for t in times]
    return SimpleNamespace(id='cam', base_path='images/cam', images=deque(images), calibration=None)


async def _request(camera: Any, time: int, *, shrink: float = 1.0, max_dimension: float | None = None,
                   fast: bool = True) -> bytes | None:
    return await _try_get_jpeg(camera, str(float(time + 1)), shrink=shrink, max_dimension=max_dimension,
                               undistort=False, fast=fast, compression=90)


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
    assert all(result is not None for result in results[:MAX_CONCURRENT_ENCODINGS])
    assert results[MAX_CONCURRENT_ENCODINGS:] == [None] * MAX_CONCURRENT_ENCODINGS


async def test_a_frame_that_is_gone_is_answered_without_waiting_for_a_slot(encoder: _Encoder) -> None:
    camera = _camera(range(MAX_CONCURRENT_ENCODINGS))
    busy = [asyncio.create_task(_request(camera, t)) for t in range(MAX_CONCURRENT_ENCODINGS)]
    await _settle()

    assert await asyncio.wait_for(_request(camera, 99), timeout=1.0) is None

    encoder.release.set()
    await asyncio.gather(*busy)


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


async def test_a_client_waiting_for_a_cancelled_encoding_encodes_the_image_itself(encoder: _Encoder) -> None:
    camera = _camera(range(1))
    owner = asyncio.create_task(_request(camera, 0))
    await _settle()
    waiter = asyncio.create_task(_request(camera, 0))
    await _settle()

    owner.cancel()
    await _settle()
    encoder.release.set()

    assert await waiter is not None
    assert len(encoder.arrays) == 2


async def test_a_cancelled_waiter_leaves_the_shared_encoding_to_the_others(encoder: _Encoder) -> None:
    camera = _camera(range(1))
    owner = asyncio.create_task(_request(camera, 0))
    await _settle()
    leaving = asyncio.create_task(_request(camera, 0))
    staying = asyncio.create_task(_request(camera, 0))
    await _settle()

    leaving.cancel()
    await _settle()
    encoder.release.set()

    assert await owner is not None
    assert await staying is not None
    assert len(encoder.arrays) == 1


async def test_no_image_is_not_cached(encoder: _Encoder) -> None:
    """``run.cpu_bound`` gives ``None`` while the server stops; the next request must encode again."""
    encoder.release.set()
    encoder.result = None
    camera = _camera(range(1))
    assert await _request(camera, 0) is None

    encoder.result = b'jpeg'
    assert await _request(camera, 0) is not None
    assert len(encoder.arrays) == 2


def test_a_reset_frees_the_slots_for_another_event_loop_and_empties_the_cache(encoder: _Encoder) -> None:
    camera = _camera(range(MAX_CONCURRENT_ENCODINGS + 1))

    async def burst() -> list[bytes | None]:
        encoder.release = asyncio.Event()
        requests = [asyncio.create_task(_request(camera, t)) for t in range(MAX_CONCURRENT_ENCODINGS + 1)]
        await _settle()  # NOTE: the last request waits for a slot, which binds the slots to this event loop
        encoder.release.set()
        return await asyncio.gather(*requests)

    assert None not in asyncio.run(burst())
    assert len(encoder.arrays) == MAX_CONCURRENT_ENCODINGS + 1

    image_route.reset_encodings()

    assert None not in asyncio.run(burst())
    assert len(encoder.arrays) == 2 * (MAX_CONCURRENT_ENCODINGS + 1)


@pytest.fixture
def encodings_of_an_earlier_test() -> None:
    image_route._encoding.cache[('earlier test',)] = b'jpeg'


async def test_an_integration_test_starts_without_the_encodings_of_an_earlier_test(
        encodings_of_an_earlier_test: None, rosys_integration: None) -> None:
    assert not image_route._encoding.cache


async def test_a_fast_shrink_sends_only_the_kept_pixels_to_the_worker(encoder: _Encoder) -> None:
    encoder.release.set()
    camera = _camera(range(1), height=60, width=90)

    await _request(camera, 0, shrink=3)

    assert encoder.arrays[0].shape == (20, 30, 3)
    assert np.shares_memory(encoder.arrays[0], camera.images[0].array)  # a view; pickling copies only its pixels
    assert len(pickle.dumps(encoder.arrays[0])) < camera.images[0].array.nbytes / 2


@pytest.mark.parametrize('shrink', [1.0, 1.5, 3.0])
@pytest.mark.parametrize('fast', [True, False])
@pytest.mark.parametrize('max_dimension', [None, 7.0])
async def test_the_route_encodes_the_same_jpeg_as_processing_the_whole_image(
        monkeypatch: pytest.MonkeyPatch, shrink: float, fast: bool, max_dimension: float | None) -> None:
    async def encode_like_a_worker(callback: Any, *args: Any) -> Any:
        return callback(*pickle.loads(pickle.dumps(args)))  # NOTE: the worker receives what pickling keeps
    monkeypatch.setattr(image_route.run, 'cpu_bound', encode_like_a_worker)
    monkeypatch.setattr(image_route, '_encoding', image_route._Encoding())
    array = np.random.default_rng(1).integers(0, 255, (12, 18, 3), dtype=np.uint8)
    camera = _camera(range(1), array=array)
    image = camera.images[0]

    jpeg = await _request(camera, 0, shrink=shrink, max_dimension=max_dimension, fast=fast)

    shrink_from_max = max(image.size.width, image.size.height) / max_dimension if max_dimension else shrink
    assert jpeg == _process(image, None, max(1, shrink, shrink_from_max), False, fast, 90, None, ImageRotation.NONE)
