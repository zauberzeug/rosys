import asyncio
from collections.abc import Awaitable, Callable

import httpx
import pytest

from rosys.vision.mjpeg_camera.mjpeg_device import MjpegDevice, _auth_for_challenge, parse_capture_timestamp


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


def test_auth_for_challenge_returns_digest_auth_for_digest_challenge():
    assert isinstance(_auth_for_challenge('Digest realm="cam", nonce="abc"', 'user', 'secret'), httpx.DigestAuth)


def test_auth_for_challenge_returns_basic_auth_for_basic_challenge():
    assert isinstance(_auth_for_challenge('Basic realm="cam"', 'user', 'secret'), httpx.BasicAuth)


def test_auth_for_challenge_falls_back_to_basic_auth_for_empty_challenge():
    assert isinstance(_auth_for_challenge('', 'user', 'secret'), httpx.BasicAuth)


def test_auth_for_challenge_falls_back_to_basic_auth_for_unknown_scheme():
    assert isinstance(_auth_for_challenge('NTLM TlRMTVNTUAABAAAAB4IIog==', 'user', 'secret'), httpx.BasicAuth)


def test_auth_for_challenge_uses_first_scheme_in_multi_scheme_header():
    assert isinstance(_auth_for_challenge('Digest realm="a", Basic realm="b"', 'user', 'secret'), httpx.DigestAuth)


def _make_device(
    monkeypatch: pytest.MonkeyPatch,
    *,
    username: str | None = None,
    password: str | None = None,
    on_new_image_data: Callable[[bytes, float], Awaitable | None] | None = None,
) -> MjpegDevice:
    monkeypatch.setattr('rosys.vision.mjpeg_camera.mjpeg_device.on_startup', lambda _handler: None)
    callback = on_new_image_data if on_new_image_data is not None else (lambda _image, _timestamp: None)
    return MjpegDevice(
        mac='7a:7a:21:00:00:00',
        ip='127.0.0.1',
        username=username,
        password=password,
        on_new_image_data=callback,
    )


def _patch_async_client_transport(monkeypatch: pytest.MonkeyPatch, transport: httpx.MockTransport) -> None:
    original_async_client = httpx.AsyncClient

    def create_client(*args, **kwargs) -> httpx.AsyncClient:
        kwargs.setdefault('transport', transport)
        return original_async_client(*args, **kwargs)

    monkeypatch.setattr('rosys.vision.mjpeg_camera.mjpeg_device.httpx.AsyncClient', create_client)


def _mjpeg_payload() -> bytes:
    header = b'--frame\r\nContent-Type: image/jpeg\r\n\r\n'
    frame = b'\xff\xd8fake-jpeg\xff\xd9'
    return header + frame


async def _run_capture(
    monkeypatch: pytest.MonkeyPatch,
    handler: Callable[[httpx.Request], httpx.Response],
    *,
    username: str | None = None,
    password: str | None = None,
) -> list[bytes]:
    """Run one capture task against a mocked camera and return the received frames."""
    received: list[bytes] = []

    async def on_new_image_data(image: bytes, timestamp: float) -> None:
        received.append(image)

    _patch_async_client_transport(monkeypatch, httpx.MockTransport(handler))
    device = _make_device(monkeypatch, username=username, password=password, on_new_image_data=on_new_image_data)
    await asyncio.wait_for(device.run_capture_task(), timeout=1.0)
    return received


async def test_capture_works_with_digest_auth_camera(monkeypatch: pytest.MonkeyPatch) -> None:
    def handler(request: httpx.Request) -> httpx.Response:
        if request.headers.get('authorization', '').startswith('Digest '):
            return httpx.Response(200, content=_mjpeg_payload(), request=request)
        return httpx.Response(401, headers={'www-authenticate': 'Digest realm="cam", nonce="abc"'}, request=request)

    received = await _run_capture(monkeypatch, handler, username='user', password='secret')
    assert len(received) == 1


async def test_capture_works_with_basic_auth_camera(monkeypatch: pytest.MonkeyPatch) -> None:
    def handler(request: httpx.Request) -> httpx.Response:
        if request.headers.get('authorization', '').startswith('Basic '):
            return httpx.Response(200, content=_mjpeg_payload(), request=request)
        return httpx.Response(401, headers={'www-authenticate': 'Basic realm="cam"'}, request=request)

    received = await _run_capture(monkeypatch, handler, username='user', password='secret')
    assert len(received) == 1


async def test_capture_does_not_send_credentials_unless_challenged(monkeypatch: pytest.MonkeyPatch) -> None:
    def handler(request: httpx.Request) -> httpx.Response:
        assert request.headers.get('authorization') is None
        return httpx.Response(200, content=_mjpeg_payload(), request=request)

    received = await _run_capture(monkeypatch, handler, username='user', password='secret')
    assert len(received) == 1


async def test_capture_works_without_credentials(monkeypatch: pytest.MonkeyPatch) -> None:
    def handler(request: httpx.Request) -> httpx.Response:
        assert request.headers.get('authorization') is None
        return httpx.Response(200, content=_mjpeg_payload(), request=request)

    received = await _run_capture(monkeypatch, handler)
    assert len(received) == 1


async def test_capture_gives_up_when_credentials_are_rejected(monkeypatch: pytest.MonkeyPatch) -> None:
    def handler(request: httpx.Request) -> httpx.Response:
        return httpx.Response(401, headers={'www-authenticate': 'Basic realm="cam"'}, request=request)

    received = await _run_capture(monkeypatch, handler, username='user', password='wrong')
    assert not received


async def test_capture_gives_up_on_non_401_errors(monkeypatch: pytest.MonkeyPatch) -> None:
    def handler(request: httpx.Request) -> httpx.Response:
        return httpx.Response(503, request=request)

    received = await _run_capture(monkeypatch, handler, username='user', password='secret')
    assert not received
