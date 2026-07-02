import asyncio
from collections.abc import Awaitable, Callable

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


async def test_auth_for_challenge_returns_digest_auth_for_digest_challenge(monkeypatch: pytest.MonkeyPatch) -> None:
    device = _make_device(monkeypatch, username='user', password='secret')

    auth = device._auth_for_challenge('Digest realm="cam", nonce="abc"', 'user',
                                      'secret')  # pylint: disable=protected-access

    assert isinstance(auth, httpx.DigestAuth)


async def test_auth_for_challenge_returns_basic_auth_for_basic_challenge(monkeypatch: pytest.MonkeyPatch) -> None:
    device = _make_device(monkeypatch, username='user', password='secret')

    auth = device._auth_for_challenge('Basic realm="cam"', 'user', 'secret')  # pylint: disable=protected-access

    assert isinstance(auth, httpx.BasicAuth)


async def test_auth_for_challenge_returns_basic_auth_for_empty_challenge(monkeypatch: pytest.MonkeyPatch) -> None:
    device = _make_device(monkeypatch, username='user', password='secret')

    auth = device._auth_for_challenge('', 'user', 'secret')  # pylint: disable=protected-access

    assert isinstance(auth, httpx.BasicAuth)


async def test_auth_for_challenge_returns_basic_auth_for_unknown_scheme(monkeypatch: pytest.MonkeyPatch) -> None:
    device = _make_device(monkeypatch, username='user', password='secret')

    auth = device._auth_for_challenge(
        'NTLM TlRMTVNTUAABAAAAB4IIogAAAAAAAAAAAAAAAAAAAAAGAbEdAAAADw==', 'user', 'secret')  # pylint: disable=protected-access

    assert isinstance(auth, httpx.BasicAuth)


async def test_auth_for_challenge_uses_first_scheme_in_multi_scheme_header(monkeypatch: pytest.MonkeyPatch) -> None:
    device = _make_device(monkeypatch, username='user', password='secret')

    auth = device._auth_for_challenge('Digest realm="a", Basic realm="b"', 'user',
                                      'secret')  # pylint: disable=protected-access

    assert isinstance(auth, httpx.DigestAuth)


async def test_run_capture_task_retries_with_digest_auth_after_401(monkeypatch: pytest.MonkeyPatch) -> None:
    """A fresh httpx.DigestAuth never preemptively attaches credentials: even once our own
    retry loop selects DigestAuth from the first 401, that auth object still runs its own
    unauthenticated-then-401-then-authenticated cycle internally (it has no cached challenge
    yet), so the full exchange is 3 requests, not 2. This is unavoidable without reaching into
    httpx's private DigestAuth challenge cache, and matches this class's pre-existing (and the
    original PR's) request count for Digest cameras -- only the Basic and no-challenge cases
    were actually reduced to their minimum by this fix.
    """
    requests: list[httpx.Request] = []
    received_frames: list[tuple[bytes, float]] = []

    async def on_new_image_data(image: bytes, timestamp: float) -> None:
        received_frames.append((image, timestamp))

    async def handler(request: httpx.Request) -> httpx.Response:
        requests.append(request)
        authorization = request.headers.get('authorization')
        if len(requests) in (1, 2):
            assert authorization is None
            return httpx.Response(401, headers={'www-authenticate': 'Digest realm="cam", nonce="abc"'}, request=request)
        if len(requests) == 3:
            assert authorization is not None
            assert authorization.startswith('Digest ')
            return httpx.Response(200, content=_mjpeg_payload(), request=request)
        pytest.fail('Expected exactly three requests for digest auth retry flow')

    transport = httpx.MockTransport(handler)
    _patch_async_client_transport(monkeypatch, transport)
    device = _make_device(monkeypatch, username='user', password='secret', on_new_image_data=on_new_image_data)

    await asyncio.wait_for(device.run_capture_task(), timeout=1.0)

    assert len(requests) == 3
    assert len(received_frames) == 1


async def test_run_capture_task_retries_with_basic_auth_after_401(monkeypatch: pytest.MonkeyPatch) -> None:
    requests: list[httpx.Request] = []
    received_frames: list[tuple[bytes, float]] = []

    async def on_new_image_data(image: bytes, timestamp: float) -> None:
        received_frames.append((image, timestamp))

    async def handler(request: httpx.Request) -> httpx.Response:
        requests.append(request)
        authorization = request.headers.get('authorization')
        if len(requests) == 1:
            assert authorization is None
            return httpx.Response(401, headers={'www-authenticate': 'Basic realm="cam"'}, request=request)
        if len(requests) == 2:
            assert authorization is not None
            assert authorization.startswith('Basic ')
            return httpx.Response(200, content=_mjpeg_payload(), request=request)
        pytest.fail('Expected exactly two requests for basic auth retry flow')

    transport = httpx.MockTransport(handler)
    _patch_async_client_transport(monkeypatch, transport)
    device = _make_device(monkeypatch, username='user', password='secret', on_new_image_data=on_new_image_data)

    await asyncio.wait_for(device.run_capture_task(), timeout=1.0)

    assert len(requests) == 2
    assert len(received_frames) == 1


async def test_run_capture_task_does_not_send_auth_without_401_challenge(monkeypatch: pytest.MonkeyPatch) -> None:
    requests: list[httpx.Request] = []
    received_frames: list[tuple[bytes, float]] = []

    async def on_new_image_data(image: bytes, timestamp: float) -> None:
        received_frames.append((image, timestamp))

    async def handler(request: httpx.Request) -> httpx.Response:
        requests.append(request)
        assert request.headers.get('authorization') is None
        return httpx.Response(200, content=_mjpeg_payload(), request=request)

    transport = httpx.MockTransport(handler)
    _patch_async_client_transport(monkeypatch, transport)
    device = _make_device(monkeypatch, username='user', password='secret', on_new_image_data=on_new_image_data)

    await asyncio.wait_for(device.run_capture_task(), timeout=1.0)

    assert len(requests) == 1
    assert len(received_frames) == 1


async def test_run_capture_task_without_credentials_sends_single_unauthenticated_request(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    requests: list[httpx.Request] = []
    received_frames: list[tuple[bytes, float]] = []

    async def on_new_image_data(image: bytes, timestamp: float) -> None:
        received_frames.append((image, timestamp))

    async def handler(request: httpx.Request) -> httpx.Response:
        requests.append(request)
        assert request.headers.get('authorization') is None
        return httpx.Response(200, content=_mjpeg_payload(), request=request)

    transport = httpx.MockTransport(handler)
    _patch_async_client_transport(monkeypatch, transport)
    device = _make_device(monkeypatch, on_new_image_data=on_new_image_data)

    await asyncio.wait_for(device.run_capture_task(), timeout=1.0)

    assert len(requests) == 1
    assert len(received_frames) == 1


async def test_run_capture_task_stops_after_second_401(monkeypatch: pytest.MonkeyPatch) -> None:
    requests: list[httpx.Request] = []
    received_frames: list[tuple[bytes, float]] = []

    async def on_new_image_data(image: bytes, timestamp: float) -> None:
        received_frames.append((image, timestamp))

    async def handler(request: httpx.Request) -> httpx.Response:
        requests.append(request)
        if len(requests) == 1:
            assert request.headers.get('authorization') is None
            return httpx.Response(401, headers={'www-authenticate': 'Basic realm="cam"'}, request=request)
        if len(requests) == 2:
            assert request.headers.get('authorization', '').startswith('Basic ')
            return httpx.Response(401, headers={'www-authenticate': 'Basic realm="cam"'}, request=request)
        pytest.fail('Expected no third request after repeated 401 responses')

    transport = httpx.MockTransport(handler)
    _patch_async_client_transport(monkeypatch, transport)
    device = _make_device(monkeypatch, username='user', password='wrong', on_new_image_data=on_new_image_data)

    await asyncio.wait_for(device.run_capture_task(), timeout=1.0)

    assert len(requests) == 2
    assert not received_frames


async def test_run_capture_task_does_not_retry_non_401_errors(monkeypatch: pytest.MonkeyPatch) -> None:
    requests: list[httpx.Request] = []
    received_frames: list[tuple[bytes, float]] = []

    async def on_new_image_data(image: bytes, timestamp: float) -> None:
        received_frames.append((image, timestamp))

    async def handler(request: httpx.Request) -> httpx.Response:
        requests.append(request)
        return httpx.Response(503, request=request)

    transport = httpx.MockTransport(handler)
    _patch_async_client_transport(monkeypatch, transport)
    device = _make_device(monkeypatch, username='user', password='secret', on_new_image_data=on_new_image_data)

    await asyncio.wait_for(device.run_capture_task(), timeout=1.0)

    assert len(requests) == 1
    assert not received_frames
