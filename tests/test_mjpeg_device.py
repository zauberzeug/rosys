from collections.abc import Callable

import httpx
import pytest

from rosys.vision.mjpeg_camera.mjpeg_device import open_stream, parse_capture_timestamp


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


async def _negotiate_stream(handler: Callable[[httpx.Request], httpx.Response],
                            *,
                            username: str | None = None,
                            password: str | None = None) -> httpx.Response | None:
    """Open a stream against a mocked camera and return the negotiated response."""
    async with httpx.AsyncClient(transport=httpx.MockTransport(handler)) as client:
        async with open_stream(client, 'http://127.0.0.1/stream', username, password) as result:
            return result.response


@pytest.mark.parametrize('challenge, auth_prefix', [
    ('Digest realm="cam", nonce="abc"', 'Digest '),
    ('Basic realm="cam"', 'Basic '),
])
async def test_answers_challenge_with_matching_auth(challenge: str, auth_prefix: str) -> None:
    def handler(request: httpx.Request) -> httpx.Response:
        if request.headers.get('authorization', '').startswith(auth_prefix):
            return httpx.Response(200)
        return httpx.Response(401, headers={'www-authenticate': challenge})

    response = await _negotiate_stream(handler, username='user', password='secret')
    assert response is not None and response.status_code == 200


async def test_sends_no_credentials_when_not_challenged() -> None:
    def handler(request: httpx.Request) -> httpx.Response:
        assert 'authorization' not in request.headers
        return httpx.Response(200)

    response = await _negotiate_stream(handler, username='user', password='secret')
    assert response is not None and response.status_code == 200


async def test_sends_no_credentials_without_username_and_password() -> None:
    def handler(request: httpx.Request) -> httpx.Response:
        assert 'authorization' not in request.headers
        return httpx.Response(401, headers={'www-authenticate': 'Basic realm="cam"'})

    response = await _negotiate_stream(handler)
    assert response is None
