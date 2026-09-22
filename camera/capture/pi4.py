"""Raspberry Pi 4B capture backend using Picamera2.

Mirrors the original main.py behaviour: Picamera2 is configured with the
"RGB888" preview format, and the captured array is consumed directly by the
downstream HSV pipeline (which treats it as BGR, as the original code did).

IMX219 on RACOON is mounted upside-down; 180° correction is applied in
software (``frame_post.apply_orientation``), not via libcamera Transform,
which is unreliable across Pi OS / Picamera2 versions.
"""

from picamera2 import Picamera2

from camera import debug
from camera.frame_post import _flip180_enabled, postprocess_frame
from camera.sensor import SENSOR_PROFILES

# Fallback when the sensor model is unknown.
_DEFAULT_CAPTURE_SIZE = (640, 480)


def _camera_info():
    return Picamera2.global_camera_info()


def _detect_sensor_model(camera_info):
    """Best-effort sensor id from libcamera metadata (imx219, ov5647, ...)."""
    model = str(camera_info.get("Model", "")).lower()
    for name in ("imx219", "ov5647", "imx477", "imx708"):
        if name in model:
            return name
    return None


def _resolve_capture_size(settings, sensor_model):
    """Returns capture width/height.

    ``threshold.json`` may override with ``frameWidth`` / ``frameHeight``.
    Otherwise use the sensor profile's full-FOV default (IMX219: 1640x1232).
    """
    if "frameWidth" in settings and "frameHeight" in settings:
        return int(settings["frameWidth"]), int(settings["frameHeight"])

    profile = SENSOR_PROFILES.get(sensor_model or "", {})
    size = profile.get("default_capture_size")
    if size:
        return int(size[0]), int(size[1])
    return _DEFAULT_CAPTURE_SIZE


class Pi4Capture:
    def __init__(self, settings=None):
        if settings is None:
            settings = {}

        cameras = _camera_info()
        if not cameras:
            raise IOError(
                "No MIPI camera detected by libcamera. "
                "Check the CSI cable and config.txt overlay "
                "(e.g. dtoverlay=imx219,cam0 or cam1). "
                "Run scripts/select-pi4-camera.sh to auto-try overlays."
            )

        self._settings = settings
        self._sensor_model = _detect_sensor_model(cameras[0])

        self.cap = Picamera2()
        width, height = _resolve_capture_size(settings, self._sensor_model)
        config = self.cap.create_preview_configuration(
            main={"size": (width, height), "format": "RGB888"},
        )
        self.cap.configure(config)
        self.cap.start()

        self._width = width
        self._height = height
        flip = _flip180_enabled(settings, self._sensor_model)
        debug.log(
            f"Pi4 (Picamera2) capture started: {self._sensor_model or 'unknown'} "
            f"{self._width}x{self._height} flip180={flip}"
        )

    def read(self):
        frame = self.cap.capture_array()
        if frame is None:
            return False, None
        # Picamera2 labels the stream RGB888, but the numpy buffer is already in
        # channel order that matches OpenCV BGR on Pi OS / libcamera builds we
        # target. Do not cvtColor here — RGB2BGR swaps R/B and makes orange look blue.
        frame = postprocess_frame(
            frame,
            self._settings,
            self._sensor_model,
            orient=True,
        )
        return True, frame

    def release(self):
        self.cap.close()
        debug.log("Pi4 video capture released.")
