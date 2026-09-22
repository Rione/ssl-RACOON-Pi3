"""Post-processing for Rock 5A V4L2 frames (white balance and orientation).

Sensor-side horizontal/vertical flip on IMX219 breaks Bayer demosaicing on the
Rockchip ISP and produces a green cast. Orientation is corrected in software
instead. IMX219 also needs per-channel gain correction; the driver exposes no
AWB controls.
"""

import os

import cv2
import numpy as np

from camera.sensor import SENSOR_PROFILES


def _bool_setting(settings, key, env_key, default=False):
    if settings and key in settings and settings[key] is not None:
        value = settings[key]
    elif env_key in os.environ:
        value = os.environ[env_key]
    else:
        return default
    return str(value).strip().lower() in ("1", "true", "yes", "on")


def _parse_color_gains(settings, sensor_model):
    """Returns BGR gain multipliers, or ``None`` to skip colour correction."""
    # IMX219 colour gains in SENSOR_PROFILES target Rock5A/Rockchip ISP only.
    if os.environ.get("RACOON_BOARD", "").strip().lower() == "pi4":
        return None

    if settings and settings.get("cameraColorGains") is not None:
        raw = str(settings["cameraColorGains"])
    elif "CAMERA_COLOR_GAINS" in os.environ:
        raw = os.environ["CAMERA_COLOR_GAINS"]
    else:
        profile = SENSOR_PROFILES.get(sensor_model or "", {})
        gains = profile.get("color_gains")
        if gains is None:
            return None
        return np.array(gains, dtype=np.float32)

    parts = [p.strip() for p in raw.split(",") if p.strip()]
    if len(parts) != 3:
        return None
    return np.array([float(p) for p in parts], dtype=np.float32)


def apply_color_gains(frame, gains_bgr):
    corrected = frame.astype(np.float32)
    corrected *= gains_bgr.reshape(1, 1, 3)
    return np.clip(corrected, 0, 255).astype(np.uint8)


def _flip180_enabled(settings, sensor_model):
    """Returns whether to apply a 180° software rotation.

    ``threshold.json`` / ``CAMERA_FLIP180`` override the per-sensor default in
    ``SENSOR_PROFILES`` (IMX219: on, OV5647: off).
    """
    if settings and "cameraFlip180" in settings and settings["cameraFlip180"] is not None:
        return _bool_setting(settings, "cameraFlip180", "CAMERA_FLIP180")
    if "CAMERA_FLIP180" in os.environ:
        return _bool_setting({}, "cameraFlip180", "CAMERA_FLIP180")
    profile = SENSOR_PROFILES.get(sensor_model or "", {})
    return bool(profile.get("default_flip180", False))


def apply_orientation(frame, settings, sensor_model=None):
    if _flip180_enabled(settings, sensor_model):
        return cv2.rotate(frame, cv2.ROTATE_180)

    out = frame
    if _bool_setting(settings, "cameraHFlip", "CAMERA_HFLIP"):
        out = cv2.flip(out, 1)
    if _bool_setting(settings, "cameraVFlip", "CAMERA_VFLIP"):
        out = cv2.flip(out, 0)
    return out


def postprocess_frame(frame, settings=None, sensor_model=None, orient=True):
    """Applies sensor-specific colour correction and software orientation."""
    settings = settings or {}

    gains = _parse_color_gains(settings, sensor_model)
    if gains is not None:
        frame = apply_color_gains(frame, gains)

    if orient:
        frame = apply_orientation(frame, settings, sensor_model)
    return frame
