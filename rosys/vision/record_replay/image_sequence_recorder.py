"""High-precision image sequence recorder for timestamped image capture.

This module provides an efficient recording pipeline that produces accurately
timestamped image sequences, suitable for offline replay and analysis.
"""
from __future__ import annotations

import asyncio
import bisect
import json
import logging
from collections import deque
from dataclasses import dataclass, field
from datetime import datetime
from pathlib import Path
from typing import TYPE_CHECKING

import aiofiles
import aiofiles.os

from ... import rosys, run
from ..image import Image
from ..image_processing import encode_image_as_jpeg
from .constants import TIME_FORMAT

if TYPE_CHECKING:
    from ..camera_provider import CameraProvider

log = logging.getLogger('rosys.vision.image_sequence_recorder')


@dataclass
class FrameMetadata:
    """Metadata for a single recorded frame."""
    filename: str
    timestamp: float
    camera_id: str
    frame_index: int
    capture_latency_ms: float | None = None  # Time between capture and save start


@dataclass
class RecordingSession:
    """Information about a recording session."""
    session_id: str
    start_time: float
    camera_id: str
    output_dir: Path
    frames: list[FrameMetadata] = field(default_factory=list)
    end_time: float | None = None

    def to_dict(self) -> dict:
        return {
            'session_id': self.session_id,
            'start_time': self.start_time,
            'end_time': self.end_time,
            'camera_id': self.camera_id,
            'frame_count': len(self.frames),
            'frames': [
                {
                    'filename': f.filename,
                    'timestamp': f.timestamp,
                    'frame_index': f.frame_index,
                    'capture_latency_ms': f.capture_latency_ms,
                }
                for f in self.frames
            ],
        }


class ImageSequenceRecorder:
    """High-precision image sequence recorder with accurate timestamps.

    This recorder is designed for scenarios where timestamp accuracy is critical,
    such as synchronizing with sensor data or offline trajectory analysis.

    Features:
    - Precise timestamp recording with nanosecond-level system time
    - Separate metadata file with frame timestamps for perfect accuracy
    - Efficient async I/O with write buffering
    - Session-based organization with unique session IDs
    - Optional capture latency tracking

    Usage:
        recorder = ImageSequenceRecorder(camera_provider, recordings_dir)
        session = recorder.start_session('my_camera_id')
        # ... images are automatically recorded ...
        await recorder.stop_session('my_camera_id')
    """

    def __init__(
        self,
        camera_provider: CameraProvider,
        data_dir: Path,
        *,
        compression_quality: int = 90,
        write_metadata: bool = True,
        max_pending_writes: int = 100,
    ) -> None:
        """Initialize the image sequence recorder.

        Args:
            camera_provider: The camera provider to record images from
            data_dir: Base directory for recordings
            compression_quality: JPEG compression quality (1-100)
            write_metadata: Whether to write a metadata JSON file with precise timestamps
            max_pending_writes: Maximum number of pending write operations before blocking
        """
        self.camera_provider = camera_provider
        self.data_dir = data_dir.expanduser()
        self.compression_quality = compression_quality
        self.write_metadata = write_metadata
        self.max_pending_writes = max_pending_writes

        self._sessions: dict[str, RecordingSession] = {}
        self._write_semaphore = asyncio.Semaphore(max_pending_writes)
        self._pending_writes: dict[str, deque[asyncio.Task]] = {}
        self._frame_counters: dict[str, int] = {}

    @property
    def active_sessions(self) -> list[str]:
        """List of currently active recording session camera IDs."""
        return list(self._sessions.keys())

    def start_session(self, camera_id: str, session_id: str | None = None) -> RecordingSession:
        """Start a new recording session for a camera.

        Args:
            camera_id: The camera to record from
            session_id: Optional custom session ID (defaults to timestamp-based)

        Returns:
            The created RecordingSession object
        """
        if camera_id in self._sessions:
            log.warning('Recording session already active for camera %s', camera_id)
            return self._sessions[camera_id]

        start_time = rosys.time()
        if session_id is None:
            session_id = datetime.fromtimestamp(start_time).strftime('%Y%m%d_%H%M%S')

        # Create output directory: data_dir/camera_id/session_id/
        safe_camera_id = camera_id.replace(':', '--')
        output_dir = self.data_dir / safe_camera_id / session_id
        output_dir.mkdir(parents=True, exist_ok=True)

        session = RecordingSession(
            session_id=session_id,
            start_time=start_time,
            camera_id=camera_id,
            output_dir=output_dir,
        )

        self._sessions[camera_id] = session
        self._pending_writes[camera_id] = deque()
        self._frame_counters[camera_id] = 0

        # Subscribe to new images
        self.camera_provider.NEW_IMAGE.subscribe(self._on_new_image)

        log.info('Started recording session %s for camera %s at %s',
                 session_id, camera_id, output_dir)
        return session

    async def stop_session(self, camera_id: str) -> RecordingSession | None:
        """Stop a recording session and finalize metadata.

        Args:
            camera_id: The camera to stop recording

        Returns:
            The completed RecordingSession, or None if no session was active
        """
        if camera_id not in self._sessions:
            log.warning('No active recording session for camera %s', camera_id)
            return None

        session = self._sessions.pop(camera_id)
        session.end_time = rosys.time()

        # Wait for pending writes to complete
        pending = self._pending_writes.pop(camera_id, deque())
        if pending:
            log.debug('Waiting for %d pending writes for camera %s', len(pending), camera_id)
            await asyncio.gather(*pending, return_exceptions=True)

        self._frame_counters.pop(camera_id, None)

        # Unsubscribe if no more active sessions
        if not self._sessions:
            self.camera_provider.NEW_IMAGE.unsubscribe(self._on_new_image)

        # Write metadata file
        if self.write_metadata:
            await self._write_session_metadata(session)

        log.info('Stopped recording session %s for camera %s: %d frames recorded',
                 session.session_id, camera_id, len(session.frames))
        return session

    async def stop_all_sessions(self) -> list[RecordingSession]:
        """Stop all active recording sessions.

        Returns:
            List of completed RecordingSession objects
        """
        camera_ids = list(self._sessions.keys())
        sessions = []
        for camera_id in camera_ids:
            session = await self.stop_session(camera_id)
            if session:
                sessions.append(session)
        return sessions

    async def _on_new_image(self, image: Image) -> None:
        """Handle a new image from the camera provider."""
        if image.camera_id not in self._sessions:
            return

        session = self._sessions[image.camera_id]
        frame_index = self._frame_counters[image.camera_id]
        self._frame_counters[image.camera_id] = frame_index + 1

        # Record the time we received this image for latency tracking
        receive_time = rosys.time()
        capture_latency_ms = (receive_time - image.time) * 1000 if image.time else None

        # Generate filename with precise timestamp
        timestamp_str = datetime.fromtimestamp(image.time).strftime(TIME_FORMAT)
        filename = f'{timestamp_str}.jpg'

        # Create metadata entry
        metadata = FrameMetadata(
            filename=filename,
            timestamp=image.time,
            camera_id=image.camera_id,
            frame_index=frame_index,
            capture_latency_ms=capture_latency_ms,
        )
        session.frames.append(metadata)

        # Schedule async write
        task = asyncio.create_task(self._write_frame(session, image, filename))
        self._pending_writes[image.camera_id].append(task)

        # Clean up completed tasks
        self._cleanup_completed_tasks(image.camera_id)

    async def _write_frame(self, session: RecordingSession, image: Image, filename: str) -> None:
        """Write a frame to disk asynchronously."""
        async with self._write_semaphore:
            file_path = session.output_dir / filename
            try:
                # Encode JPEG in thread pool
                jpeg_bytes = await run.cpu_bound(
                    encode_image_as_jpeg,
                    image.array,
                    self.compression_quality,
                )
                if jpeg_bytes is None:
                    return

                # Write to disk asynchronously
                async with aiofiles.open(file_path, 'wb') as f:
                    await f.write(jpeg_bytes)

                log.debug('Saved frame %s to %s', filename, file_path)
            except Exception as e:
                log.error('Failed to save frame %s: %s', filename, e)

    def _cleanup_completed_tasks(self, camera_id: str) -> None:
        """Remove completed tasks from the pending queue."""
        pending = self._pending_writes.get(camera_id)
        if not pending:
            return

        while pending and pending[0].done():
            task = pending.popleft()
            # Check for exceptions
            if task.exception():
                log.error('Frame write task failed: %s', task.exception())

    async def _write_session_metadata(self, session: RecordingSession) -> None:
        """Write session metadata to a JSON file."""
        metadata_path = session.output_dir / 'metadata.json'
        try:
            metadata = session.to_dict()
            async with aiofiles.open(metadata_path, 'w') as f:
                await f.write(json.dumps(metadata, indent=2))
            log.debug('Wrote session metadata to %s', metadata_path)
        except Exception as e:
            log.error('Failed to write session metadata: %s', e)

    def get_session(self, camera_id: str) -> RecordingSession | None:
        """Get the active recording session for a camera."""
        return self._sessions.get(camera_id)

    def get_frame_count(self, camera_id: str) -> int:
        """Get the number of frames recorded for a camera in the current session."""
        session = self._sessions.get(camera_id)
        return len(session.frames) if session else 0


class TimestampIndex:
    """Index for efficiently looking up images by timestamp.

    This class reads the metadata.json file from an ImageSequenceRecorder session
    and provides efficient timestamp-based lookups.
    """

    def __init__(self, session_dir: Path) -> None:
        """Load timestamp index from a session directory.

        Args:
            session_dir: Path to the session directory containing metadata.json
        """
        self.session_dir = session_dir
        self.frames: list[FrameMetadata] = []
        self._timestamps: list[float] = []

        self._load_metadata()

    def _load_metadata(self) -> None:
        """Load metadata from JSON file."""
        metadata_path = self.session_dir / 'metadata.json'
        if not metadata_path.exists():
            log.warning('No metadata.json found in %s', self.session_dir)
            return

        with open(metadata_path) as f:
            data = json.load(f)

        for frame_data in data.get('frames', []):
            frame = FrameMetadata(
                filename=frame_data['filename'],
                timestamp=frame_data['timestamp'],
                camera_id=data.get('camera_id', ''),
                frame_index=frame_data['frame_index'],
                capture_latency_ms=frame_data.get('capture_latency_ms'),
            )
            self.frames.append(frame)
            self._timestamps.append(frame.timestamp)

    def find_by_timestamp(self, timestamp: float, tolerance: float = 0.1) -> FrameMetadata | None:
        """Find the frame closest to the given timestamp.

        Args:
            timestamp: Target timestamp in seconds
            tolerance: Maximum allowed time difference in seconds

        Returns:
            The closest FrameMetadata, or None if no frame within tolerance
        """
        if not self._timestamps:
            return None

        idx = bisect.bisect_left(self._timestamps, timestamp)

        candidates = []
        if idx > 0:
            candidates.append((abs(self._timestamps[idx - 1] - timestamp), idx - 1))
        if idx < len(self._timestamps):
            candidates.append((abs(self._timestamps[idx] - timestamp), idx))

        if not candidates:
            return None

        best_diff, best_idx = min(candidates)
        if best_diff <= tolerance:
            return self.frames[best_idx]
        return None

    def get_frame_path(self, frame: FrameMetadata) -> Path:
        """Get the full path to a frame's image file."""
        return self.session_dir / frame.filename

    @property
    def start_time(self) -> float | None:
        """Get the timestamp of the first frame."""
        return self._timestamps[0] if self._timestamps else None

    @property
    def end_time(self) -> float | None:
        """Get the timestamp of the last frame."""
        return self._timestamps[-1] if self._timestamps else None

    @property
    def frame_count(self) -> int:
        """Get the total number of frames."""
        return len(self.frames)
