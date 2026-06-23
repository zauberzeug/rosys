from rosys.vision.rtsp_camera.capture_time import (
    decode_abs_capture_time,
    encode_abs_capture_time,
    read_abs_capture_time,
)


def test_decode_known_vector():
    # 1900-01-01 + _NTP_UNIX_EPOCH_OFFSET seconds == Unix epoch 0, fraction 0.
    payload = (2_208_988_800 << 32).to_bytes(8, 'big')
    assert decode_abs_capture_time(payload) == 0.0


def test_encode_decode_roundtrip():
    epoch = 1718900000.123456
    decoded = decode_abs_capture_time(encode_abs_capture_time(epoch))
    assert abs(decoded - epoch) < 1e-6


def test_decode_ignores_trailing_optional_fields():
    payload = encode_abs_capture_time(1718900000.0) + b'\x00\x00\x00\x00'
    assert abs(decode_abs_capture_time(payload) - 1718900000.0) < 1e-6


class _FakeRtpBuffer:
    def __init__(self, ext: dict[int, bytes]) -> None:
        self._ext = ext

    def get_extension_onebyte_header(self, ext_id: int, _index: int):
        data = self._ext.get(ext_id)
        return (data is not None, data)


def test_read_returns_epoch_when_extension_present():
    payload = encode_abs_capture_time(1718900000.5)
    rtp = _FakeRtpBuffer({10: payload})
    assert abs(read_abs_capture_time(rtp, 10) - 1718900000.5) < 1e-6


def test_read_returns_none_when_extension_absent():
    assert read_abs_capture_time(_FakeRtpBuffer({}), 10) is None
