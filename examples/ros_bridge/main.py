#!/usr/bin/env python3
from pathlib import Path
from typing import Any, Callable, Optional

import cv2
import numpy as np
from nicegui import ui
from norospy import ROSFoxgloveClient

import rosys


class RosBridge():
    def __init__(self, websocket_url: str = "ws://localhost:8765") -> None:
        """Initialize ROS bridge with Foxglove WebSocket connection.

        Args:
            websocket_url: WebSocket URL for Foxglove connection
        """
        self.websocket_url = websocket_url
        self.client: ROSFoxgloveClient | None = None
        self._image_callback: Callable | None = None
        self._connected = False

        # Ensure images directory exists
        self.image_dir = Path('images')
        self.image_dir.mkdir(exist_ok=True)

        rosys.on_repeat(self.connect, 0.1)

    async def run(self) -> None:
        while True:
            if not self._connected:
                await self.connect()
                await rosys.sleep(0.1)
                continue
            self.client.run()
            await rosys.sleep(0.1)

    async def connect(self, max_attempts: int = 3, retry_delay: int = 2) -> bool:
        """Connect to Foxglove WebSocket server with retries."""
        if self._connected:
            return True

        for attempt in range(max_attempts):
            try:
                self.client = ROSFoxgloveClient(self.websocket_url)
                self.client.run_background()
                self._connected = True
                print("Connected to Foxglove WebSocket server")
                return True
            except ConnectionRefusedError:
                if attempt < max_attempts - 1:
                    print(f"Connection attempt {attempt + 1} failed. Retrying in {retry_delay} seconds...")
                else:
                    print("Error: Could not connect to Foxglove WebSocket server")
                    print("Please make sure:")
                    print("1. The ROS2 Foxglove bridge container is running")
                    print("2. The container has network access")
                    print("3. No other service is using the port")
        return False

    def _handle_image(self, msg, ts):
        """Internal handler for image messages."""
        print(f"Received image at timestamp: {ts}")
        print(f"Image message type: {type(msg)}")
        print(f"Image data length: {len(msg.data)}")
        print(f"Image encoding: {getattr(msg, 'encoding', 'unknown')}")

        # Convert raw BGR data to numpy array
        height = msg.height
        width = msg.width
        img_array = np.frombuffer(msg.data, dtype=np.uint8).reshape(height, width, 3)

        # Save as JPEG
        image_path = self.image_dir / f"{ts}.jpg"
        is_success, buffer = cv2.imencode('.jpg', img_array)

        if is_success:
            image_path.write_bytes(buffer.tobytes())
        else:
            print("Failed to encode image")

        # Call user callback if set
        if self._image_callback:
            self._image_callback(img_array, ts)

    def subscribe_to_images(self, topic: str, callback: Callable | None = None) -> None:
        """Subscribe to ROS image topic.

        Args:
            topic: ROS topic name
            callback: Optional callback function(image_array, timestamp)
        """
        if not self._connected:
            raise RuntimeError("Not connected to Foxglove. Call connect() first")

        self._image_callback = callback
        self.client.subscribe(topic, 'sensor_msgs/Image', self._handle_image)
        print(f"Subscribed to image topic: {topic}")

    def close(self) -> None:
        """Close the connection."""
        if self.client:
            self.client.close()
            self.client = None
        self._connected = False


class FileCounter(ui.label):
    def __init__(self) -> None:
        super().__init__()
        self.count = 0
        rosys.on_repeat(self.count_files, 1)

    def count_files(self) -> None:
        # Count files in images directory
        image_files = len(list(Path('./images').glob('*.jpg')))
        self.count = image_files
        self.text = f"Files: {self.count}"


def on_image(image_array: np.ndarray, timestamp: float) -> None:
    print(f"Custom handler: Received image at {timestamp}")


ros_bridge = RosBridge()


def subscribe_to_images():
    ros_bridge.subscribe_to_images('/test_image', callback=on_image)


ui.button('Subscribe to images', on_click=subscribe_to_images)

FileCounter()
ui.run(title='Camera Arm')
