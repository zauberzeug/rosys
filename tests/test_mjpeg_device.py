from collections.abc import Callable

import httpx
import pytest

from rosys.vision.mjpeg_camera.mjpeg_device import MjpegDevice, parse_capture_timestamp


def test_parses_x_timestamp():
    header = b'\r\n--boundary\r\nContent-Type: image/jpeg\r\nContent-Length: 1234\r\nX-Timestamp: 1718900000.123456\r\n\r\n'
    assert parse_capture_timestamp(header) == 1718900000.123456


def test_parses_x_timestamp_case_insensitively():
    header = b'--boundary\r\nx-timestamp:1718900000.5\r\n\r\n'
    assert parse_capture_timestamp(header) == 1718900000.5


def test_returns_none_without_header():
    header = b'\r\n--boundary\r\nContent-Type: image/jpeg\r\nContent-Length: 1234\r\n\r\n'
    assert parse_capture_timestamp(header) is None


def test_returns_none_for_unparsable_value():
    header = b'X-Timestamp: not-a-number\r\n\r\n'
    assert parse_capture_timestamp(header) is None


def test_uses_last_header_when_multiple_present():
    header = b'X-Timestamp: 1.0\r\n\r\n<jpeg>\r\n--boundary\r\nX-Timestamp: 2.0\r\n\r\n'
    assert parse_capture_timestamp(header) == 2.0


async def _open_stream(monkeypatch: pytest.MonkeyPatch,
                       handler: Callable[[httpx.Request], httpx.Response],
                       *,
                       username: str | None = None,
                       password: str | None = None) -> httpx.Response | None:
    """Open a device stream against a mocked camera and return the negotiated response."""
    monkeypatch.setattr('rosys.vision.mjpeg_camera.mjpeg_device.on_startup', lambda _handler: None)
    device = MjpegDevice(mac='7a:7a:21:00:00:00', ip='127.0.0.1',
                         username=username, password=password,
                         on_new_image_data=lambda _image, _timestamp: None)
    async with httpx.AsyncClient(transport=httpx.MockTransport(handler)) as client:
        async with device._open_stream(client) as response:  # pylint: disable=protected-access
            return response


async def test_answers_digest_challenge_with_digest_auth(monkeypatch: pytest.MonkeyPatch) -> None:
    def handler(request: httpx.Request) -> httpx.Response:
        if request.headers.get('authorization', '').startswith('Digest '):
            return httpx.Response(200)
        return httpx.Response(401, headers={'www-authenticate': 'Digest realm="cam", nonce="abc"'})

    response = await _open_stream(monkeypatch, handler, username='user', password='secret')
    assert response is not None and response.status_code == 200


async def test_answers_basic_challenge_with_basic_auth(monkeypatch: pytest.MonkeyPatch) -> None:
    def handler(request: httpx.Request) -> httpx.Response:
        if request.headers.get('authorization', '').startswith('Basic '):
            return httpx.Response(200)
        return httpx.Response(401, headers={'www-authenticate': 'Basic realm="cam"'})

    response = await _open_stream(monkeypatch, handler, username='user', password='secret')
    assert response is not None and response.status_code == 200


async def test_sends_no_credentials_when_not_challenged(monkeypatch: pytest.MonkeyPatch) -> None:
    def handler(request: httpx.Request) -> httpx.Response:
        assert 'authorization' not in request.headers
        return httpx.Response(200)

    response = await _open_stream(monkeypatch, handler, username='user', password='secret')
    assert response is not None and response.status_code == 200


async def test_sends_no_credentials_without_username_and_password(monkeypatch: pytest.MonkeyPatch) -> None:
    def handler(request: httpx.Request) -> httpx.Response:
        assert 'authorization' not in request.headers
        return httpx.Response(401, headers={'www-authenticate': 'Basic realm="cam"'})

    response = await _open_stream(monkeypatch, handler)
    assert response is None
