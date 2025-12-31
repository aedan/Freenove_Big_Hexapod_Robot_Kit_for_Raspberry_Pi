"""
Camera module for Freenove Hexapod Robot.
Uses MJPEG for streaming (client compatibility) and H264 for file recording.
"""

import time
import io
from threading import Condition
from typing import Optional

from picamera2 import Picamera2, Preview
from picamera2.encoders import H264Encoder, MJPEGEncoder, Quality
from picamera2.outputs import FileOutput
from libcamera import Transform

from config import get_config
from logger import get_logger

logger = get_logger()


class StreamingOutput(io.BufferedIOBase):
    """Thread-safe video frame buffer with timeout support."""

    def __init__(self):
        """Initialize the StreamingOutput class."""
        self.frame: Optional[bytes] = None
        self.condition = Condition()

    def write(self, buf: bytes) -> int:
        """
        Write a buffer to the frame and notify all waiting threads.

        Args:
            buf: Frame data to write

        Returns:
            Number of bytes written
        """
        with self.condition:
            self.frame = buf
            self.condition.notify_all()
        return len(buf)


class Camera:
    """
    Camera controller for video streaming and image capture.
    Uses MJPEG encoding for streaming, H264 for file recording.
    """

    def __init__(
        self,
        preview_size: Optional[tuple] = None,
        hflip: bool = False,
        vflip: bool = False,
        stream_size: Optional[tuple] = None
    ):
        """
        Initialize the Camera class.

        Args:
            preview_size: Size for preview mode (default from config)
            hflip: Horizontal flip
            vflip: Vertical flip
            stream_size: Size for video streaming (default from config)
        """
        self.config = get_config()

        # Use config defaults if not specified
        if preview_size is None:
            preview_size = self.config.camera.resolution
        if stream_size is None:
            stream_size = self.config.camera.resolution

        try:
            self.camera = Picamera2()
            logger.info("Camera initialized successfully")
        except IndexError:
            logger.error("No available camera device found")
            raise RuntimeError("Camera initialization failed - no device found")

        # Set image transformation
        self.transform = Transform(
            hflip=1 if hflip else 0,
            vflip=1 if vflip else 0
        )

        # Preview configuration
        preview_config = self.camera.create_preview_configuration(
            main={"size": preview_size},
            transform=self.transform
        )
        self.camera.configure(preview_config)

        # Video stream configuration
        self.stream_size = stream_size
        self.stream_config = self.camera.create_video_configuration(
            main={"size": stream_size},
            transform=self.transform
        )
        self.streaming_output = StreamingOutput()
        self.streaming = False
        self.encoder: Optional[H264Encoder] = None

        logger.info(f"Camera configured - preview: {preview_size}, stream: {stream_size}")

    def start_image(self) -> None:
        """Start the camera preview and capture."""
        try:
            self.camera.start_preview(Preview.QTGL)
            self.camera.start()
            logger.info("Camera preview started")
        except Exception as e:
            logger.error(f"Failed to start camera preview: {e}")
            raise

    def save_image(self, filename: str) -> Optional[dict]:
        """
        Capture and save an image to the specified file.

        Args:
            filename: Output file path

        Returns:
            Image metadata or None on failure
        """
        try:
            metadata = self.camera.capture_file(filename)
            logger.info(f"Image saved to {filename}")
            return metadata
        except Exception as e:
            logger.error(f"Error capturing image: {e}")
            return None

    def start_stream(self, filename: Optional[str] = None) -> None:
        """
        Start the video stream or recording.
        Uses MJPEG for streaming (client compatibility) and H264 for file recording.

        Args:
            filename: If provided, save to file; otherwise stream to buffer
        """
        if self.streaming:
            logger.warning("Stream already running")
            return

        try:
            # Stop camera if running
            if self.camera.started:
                self.camera.stop()

            # Configure camera for video
            self.camera.configure(self.stream_config)

            # Set output and encoder based on use case
            if filename:
                # Use H264 for file recording (better compression)
                self.encoder = H264Encoder(
                    bitrate=self.config.camera.bitrate,
                )
                # Set I-frame interval for better seeking
                if hasattr(self.encoder, 'intra_period'):
                    self.encoder.intra_period = self.config.camera.intra_period

                output = FileOutput(filename)
                logger.info(f"Starting H264 video recording to {filename}")
            else:
                # Use MJPEG for streaming (client expects JPEG frames)
                self.encoder = MJPEGEncoder(
                    bitrate=self.config.camera.bitrate,
                )
                output = FileOutput(self.streaming_output)
                logger.info("Starting MJPEG video streaming")

            # Start recording
            self.camera.start_recording(self.encoder, output)
            self.streaming = True
            logger.info(
                f"Camera streaming: {self.stream_size} @ {self.config.camera.framerate}fps, "
                f"bitrate={self.config.camera.bitrate}"
            )

        except Exception as e:
            logger.error(f"Failed to start camera stream: {e}")
            self.streaming = False
            raise

    def stop_stream(self) -> None:
        """Stop the video stream or recording."""
        if not self.streaming:
            return

        try:
            self.camera.stop_recording()
            self.streaming = False
            logger.info("Camera stream stopped")
        except Exception as e:
            logger.error(f"Error stopping stream: {e}")

    def get_frame(self, timeout: float = 2.0) -> Optional[bytes]:
        """
        Get the current frame from the streaming output.

        Args:
            timeout: Max seconds to wait for a frame (default 2.0)

        Returns:
            Frame data or None on timeout
        """
        with self.streaming_output.condition:
            # Wait for frame with timeout
            if not self.streaming_output.condition.wait(timeout=timeout):
                logger.warning(f"Frame timeout after {timeout}s")
                return None
            return self.streaming_output.frame

    def save_video(self, filename: str, duration: int = 10) -> None:
        """
        Save a video for the specified duration.

        Args:
            filename: Output file path
            duration: Recording duration in seconds
        """
        logger.info(f"Recording video for {duration}s to {filename}")
        self.start_stream(filename)
        time.sleep(duration)
        self.stop_stream()

    def capture_frame(self) -> Optional['numpy.ndarray']:
        """
        Capture a single frame as numpy array for OpenCV processing.
        Useful for face recognition and image analysis.

        Returns:
            BGR numpy array or None on failure
        """
        try:
            import numpy as np

            # Capture frame as numpy array
            frame = self.camera.capture_array("main")

            # Convert from RGB to BGR for OpenCV compatibility
            if frame is not None and len(frame.shape) == 3:
                # picamera2 returns RGB, OpenCV expects BGR
                frame_bgr = np.ascontiguousarray(frame[:, :, ::-1])
                return frame_bgr
            else:
                logger.warning("Invalid frame captured")
                return None

        except Exception as e:
            logger.error(f"Error capturing frame: {e}")
            return None

    def close(self) -> None:
        """Close the camera and cleanup resources."""
        logger.info("Closing camera")
        if self.streaming:
            self.stop_stream()
        try:
            self.camera.close()
            logger.info("Camera closed")
        except Exception as e:
            logger.warning(f"Error closing camera: {e}")

    def __enter__(self):
        """Context manager entry."""
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        """Context manager exit - cleanup resources."""
        self.close()
        return False


if __name__ == '__main__':
    from logger import setup_logger

    setup_logger(log_level='INFO')

    logger.info('Camera test program starting')

    with Camera() as camera:
        logger.info("View image...")
        camera.start_image()
        time.sleep(10)

        logger.info("Capture image...")
        camera.save_image(filename="image.jpg")
        time.sleep(1)

        logger.info("Stream video test...")
        camera.start_stream()
        time.sleep(3)

        logger.info("Stop video...")
        camera.stop_stream()
        time.sleep(1)

        logger.info("Save video test...")
        camera.save_video("video.h264", duration=3)
        time.sleep(1)

    logger.info("Camera test complete")
