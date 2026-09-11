from collections.abc import Callable

import httpx
import numpy as np
import pytest

from rosys.vision.mjpeg_camera.mjpeg_stream_worker import _open_stream, _parse_capture_timestamp, _split_frames
from rosys.vision.mjpeg_camera.stream_channel import (
    Frame,
    memfd_is_available,
    open_channel,
    open_memfd_channel,
    open_pickled_channel,
)


def test_parses_x_timestamp():
    header = b'\r\n--boundary\r\nContent-Type: image/jpeg\r\nContent-Length: 1234\r\nX-Timestamp: 1718900000.123456\r\n\r\n'
    assert _parse_capture_timestamp(header) == 1718900000.123456


def test_parses_x_timestamp_case_insensitively():
    header = b'--boundary\r\nx-timestamp:1718900000.5\r\n\r\n'
    assert _parse_capture_timestamp(header) == 1718900000.5


def test_returns_none_without_header():
    header = b'\r\n--boundary\r\nContent-Type: image/jpeg\r\nContent-Length: 1234\r\n\r\n'
    assert _parse_capture_timestamp(header) is None


def test_returns_none_for_unparsable_value():
    header = b'X-Timestamp: not-a-number\r\n\r\n'
    assert _parse_capture_timestamp(header) is None


def test_uses_last_header_when_multiple_present():
    header = b'X-Timestamp: 1.0\r\n\r\n<jpeg>\r\n--boundary\r\nX-Timestamp: 2.0\r\n\r\n'
    assert _parse_capture_timestamp(header) == 2.0


def test_yields_the_newest_complete_frame_per_chunk_with_its_capture_time():
    jpeg = b'\xff\xd8' + bytes(8) + b'\xff\xd9'
    stream = (b'--frame\r\nX-Timestamp: 1.5\r\n\r\n' + jpeg + b'\r\n--frame\r\nX-Timestamp: 2.0\r\n\r\n' + jpeg[:5],
              jpeg[5:] + b'\r\n--frame\r\nX-Timestamp: 3.0\r\n\r\n' + jpeg,
              b'\r\n--frame\r\nContent-Type: image/jpeg\r\n\r\n' + jpeg)
    assert list(_split_frames(stream)) == [(jpeg, 1.5), (jpeg, 3.0), (jpeg, None)]


def _negotiate_stream(handler: Callable[[httpx.Request], httpx.Response],
                      *,
                      username: str | None = None,
                      password: str | None = None) -> int:
    """Open a stream against a mocked camera and return the status of the negotiated response."""
    with httpx.Client(transport=httpx.MockTransport(handler)) as client:
        with _open_stream(client, 'http://127.0.0.1/stream', username, password) as response:
            return response.status_code


@pytest.mark.parametrize('challenge, auth_prefix', [
    ('Digest realm="cam", nonce="abc"', 'Digest '),
    ('Basic realm="cam"', 'Basic '),
])
def test_answers_challenge_with_matching_auth(challenge: str, auth_prefix: str) -> None:
    def handler(request: httpx.Request) -> httpx.Response:
        if request.headers.get('authorization', '').startswith(auth_prefix):
            return httpx.Response(200)
        return httpx.Response(401, headers={'www-authenticate': challenge})

    assert _negotiate_stream(handler, username='user', password='secret') == 200


def test_sends_no_credentials_when_not_challenged() -> None:
    def handler(request: httpx.Request) -> httpx.Response:
        assert 'authorization' not in request.headers
        return httpx.Response(200)

    assert _negotiate_stream(handler, username='user', password='secret') == 200


def test_sends_no_credentials_without_username_and_password() -> None:
    def handler(request: httpx.Request) -> httpx.Response:
        assert 'authorization' not in request.headers
        return httpx.Response(401, headers={'www-authenticate': 'Basic realm="cam"'})

    assert _negotiate_stream(handler) == 401


@pytest.mark.parametrize('open_channel_', [
    open_pickled_channel,
    pytest.param(open_memfd_channel, marks=pytest.mark.skipif(not memfd_is_available(), reason='no memfd')),
])
def test_frames_and_other_messages_survive_the_channel(open_channel_) -> None:
    receiver, sender = open_channel_()
    array = np.random.default_rng(0).integers(0, 255, size=(4, 6, 3), dtype=np.uint8)
    sender.send(Frame(array=array, capture_time=1.5))
    sender.send('not a frame')
    sender.close()

    frame = receiver.receive()
    assert isinstance(frame, Frame)
    assert frame.capture_time == 1.5
    assert np.array_equal(frame.array, array)
    assert receiver.receive() == 'not a frame'
    receiver.close()


def test_the_platform_channel_is_chosen() -> None:
    assert open_channel is (open_memfd_channel if memfd_is_available() else open_pickled_channel)
