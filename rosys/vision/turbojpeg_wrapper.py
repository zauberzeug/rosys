"""Wrapper for TurboJPEG library to normalize return types and provide consistent error handling."""
import numpy as np

try:
    from turbojpeg import TJPF_RGB, TurboJPEG  # type: ignore[import-untyped]
    PY_TURBOJPEG_AVAILABLE = True
except (ModuleNotFoundError, ImportError):
    PY_TURBOJPEG_AVAILABLE = False
    TurboJPEG = None  # type: ignore[assignment, misc]
    TJPF_RGB = 0


class TurboJpegWrapper:
    """Wrapper for TurboJPEG to normalize return types and provide consistent error handling.

    TurboJPEG.encode() can return either bytes or (bytes, int) tuple depending on parameters.
    This wrapper ensures consistent bytes return type and proper exception handling.
    """

    def __init__(self) -> None:
        self._turbo_jpeg = TurboJPEG()  # type: ignore[misc]

    def encode(self, image_array: np.ndarray, quality: int, pixel_format: int = TJPF_RGB) -> bytes:
        """Encode image to JPEG bytes.

        :param image: Image array to encode.
        :param quality: JPEG quality level (1-100).
        :param pixel_format: Pixel format constant from turbojpeg.
        :return: Encoded JPEG bytes.
        :raises RuntimeError: If encoding fails.
        """
        try:
            result = self._turbo_jpeg.encode(image_array, quality=quality, pixel_format=pixel_format)
        except (ValueError, OSError, TypeError) as e:
            raise RuntimeError(f'TurboJPEG encoding failed: {e}') from e

        if isinstance(result, tuple):
            encoded_bytes = result[0]
            if not isinstance(encoded_bytes, bytes):
                raise RuntimeError(f'Unexpected type in tuple from TurboJPEG.encode: {type(encoded_bytes)}')
            return bytes(encoded_bytes)
        if isinstance(result, bytes):
            return result

        raise RuntimeError(f'Unexpected return type from TurboJPEG.encode: {type(result)}')

    def decode(self, jpeg_bytes: bytes, pixel_format: int = TJPF_RGB) -> np.ndarray:
        """Decode JPEG bytes to NumPy array.

        :param jpeg_bytes: JPEG encoded image data.
        :param pixel_format: Pixel format constant from turbojpeg.
        :return: Decoded image as numpy array.
        :raises OSError: If decoding fails.
        """
        try:
            result = self._turbo_jpeg.decode(jpeg_bytes, pixel_format=pixel_format)
        except (ValueError, OSError, TypeError) as e:
            raise OSError(f'TurboJPEG decoding failed: {e}') from e

        if not isinstance(result, np.ndarray):
            raise RuntimeError(f'Unexpected return type from TurboJPEG.decode: {type(result)}')

        return result


TURBO_JPEG: TurboJpegWrapper | None
if PY_TURBOJPEG_AVAILABLE:
    try:
        TURBO_JPEG = TurboJpegWrapper()
    except (RuntimeError, OSError):
        # native library is not available
        TURBO_JPEG = None
else:
    # python wrapper library is not available
    TURBO_JPEG = None
