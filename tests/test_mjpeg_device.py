from rosys.vision.mjpeg_camera.mjpeg_device import parse_capture_timestamp


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
