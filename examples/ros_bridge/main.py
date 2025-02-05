#!/usr/bin/env python3
import base64
from pathlib import Path
from typing import Callable

import cv2
import numpy as np
from fastapi import Response
from nicegui import app, ui
from norospy import ROSFoxgloveClient

import rosys
from rosys.vision.image_route import _process


class RosBridge:
    def __init__(self, websocket_url: str = "ws://localhost:8765") -> None:
        """Initialize ROS bridge with Foxglove WebSocket connection.

        Args:
            websocket_url: WebSocket URL for Foxglove connection
        """
        self.websocket_url = websocket_url
        self.client: ROSFoxgloveClient | None = None
        self._image_callback: Callable | None = None
        self._connected = False
        rosys.on_repeat(self.connect, 0.1)
        # rosys.on_startup(self.run)

    async def run(self) -> None:
        # TODO: not working
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

        for _ in range(max_attempts):
            try:
                self.client = ROSFoxgloveClient(self.websocket_url)
                self.client.run_background()
                self._connected = True
                print("Connected to Foxglove WebSocket server")
                return True
            except ConnectionRefusedError:
                await rosys.sleep(retry_delay)
        return False

    @staticmethod
    def convert(bgr_image: np.ndarray) -> bytes:
        _, jpeg_image = cv2.imencode('.jpg', bgr_image)
        jpeg_image_bytes = jpeg_image.tobytes()
        return jpeg_image_bytes

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

        jpeg_image_bytes = self.convert(img_array)
        new_image = rosys.vision.Image(
            camera_id='test',
            size=rosys.vision.ImageSize(width=width, height=height),
            time=ts,
            data=jpeg_image_bytes,
        )

        # Call user callback if set
        if self._image_callback:
            self._image_callback(new_image, ts)

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


image: rosys.vision.Image | None = None
placeholder = Response(
    content=cv2.imencode('.jpg', np.zeros((480, 640, 3), dtype=np.uint8))[1].tobytes(),
    media_type='image/jpeg'
)
with ui.card().tight():
    image_view = ui.interactive_image('/image?1')


def on_image(new_image: rosys.vision.Image, timestamp: float) -> None:
    print(f'Received image at timestamp: {timestamp}')
    global image
    image = new_image
    image_view.set_source(f'/image?{image.time}')


@app.get('/image')
async def grab_frame() -> Response:
    if image is None:
        return placeholder
    data = _process(image.data, calibration=None, shrink=1, fast=False, undistort=False, compression=100)
    return Response(content=data, media_type='image/jpeg')


ros_bridge = RosBridge()


def subscribe_to_images():
    ros_bridge.subscribe_to_images('/test_image', callback=on_image)


ui.button('Subscribe to images', on_click=subscribe_to_images)

ui.run(title='Camera Arm')
