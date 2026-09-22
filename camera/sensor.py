"""MIPI CSI sensor tuning for Rock 5A (V4L2 subdev).

Supports both the OV5647 (Raspberry Pi Camera v1.3) and the IMX219 (Raspberry
Pi Camera v2). The connected sensor is auto-detected from the V4L2 subdev names
exposed under ``/sys/class/video4linux``, and exposure/gain controls are applied
with sensor-specific control names and ranges.

Manual exposure/gain avoids blown-out highlights on the ball under bright
overhead lighting. Values can be overridden via threshold.json or env vars.
"""

import glob
import os
import subprocess

from camera import debug

DEFAULT_SUBDEV = "/dev/v4l-subdev2"
DEFAULT_EXPOSURE = 900
DEFAULT_GAIN = 80

KNOWN_SENSORS = ("imx219", "ov5647")

# Per-sensor profiles. Exposure/gain ranges follow the Rockchip vendor kernel
# (6.1) V4L2 subdev controls for each sensor. ``gain_ctrl`` is the control name
# that actually drives brightness for that sensor.
#
# IMX219 note: brightness is driven by the combined ``gain`` control (max 43663,
# analogue + digital), not ``analogue_gain`` (max 2816 ~= 1 stop only). Exposure
# barely affects brightness in the default 30fps mode, so it is kept low to
# minimise motion blur on a fast-moving ball and ``gain`` does the lifting.
SENSOR_PROFILES = {
    "ov5647": {
        "gain_ctrl": "analogue_gain",
        "exposure": (4, 1964),
        "gain": (16, 1023),
        "default_exposure": 900,
        "default_gain": 80,
        "default_flip180": False,
        # Pi 4 / libcamera: 1296x972 is binned full field of view.
        "default_capture_size": (1296, 972),
    },
    "imx219": {
        "gain_ctrl": "gain",
        "exposure": (1, 4095),
        "gain": (256, 43663),
        "default_exposure": 1000,
        # Keep gain moderate: values above ~7000 saturate highlights in bright
        # overhead lighting (raw clip >10%% before colour correction).
        "default_gain": 5000,
        # Empirical BGR multipliers for Rockchip ISP + IMX219 (no driver AWB).
        "color_gains": (1.15, 0.78, 1.12),
        # RACOON mount: IMX219 module is upside-down; OV5647 is not.
        "default_flip180": True,
        # Pi 4 / libcamera: 1640x1232 is 2x2-binned full field of view.
        # 640x480 uses a centre window and crops heavily.
        "default_capture_size": (1640, 1232),
    },
}


def _subdev_name(node):
    base = os.path.basename(node)
    try:
        with open(f"/sys/class/video4linux/{base}/name") as f:
            return f.read().strip()
    except OSError:
        return ""


def detect_sensor(preferred_subdev=None):
    """Returns ``(subdev_path, model)`` for the connected sensor.

    ``preferred_subdev`` (if its name matches a known sensor) takes priority;
    otherwise every ``/dev/v4l-subdev*`` is scanned. ``model`` is ``None`` when
    no known sensor is found.
    """
    if preferred_subdev:
        name = _subdev_name(preferred_subdev).lower()
        for model in KNOWN_SENSORS:
            if model in name:
                return preferred_subdev, model
        # Preferred node didn't match a known sensor; fall back to scanning.

    for node in sorted(glob.glob("/dev/v4l-subdev*")):
        name = _subdev_name(node).lower()
        for model in KNOWN_SENSORS:
            if model in name:
                return node, model

    return (preferred_subdev or DEFAULT_SUBDEV), None


def _clamp(value, low, high):
    return max(low, min(value, high))


def _int_setting(settings, key, env_key, default):
    if settings and key in settings and settings[key] is not None:
        return int(settings[key])
    if env_key in os.environ:
        return int(os.environ[env_key])
    return default


def _build_ctrl(model, exposure, gain, auto_exposure):
    """Returns the ``v4l2-ctl --set-ctrl`` value, or ``None`` to skip."""
    if model == "imx219":
        # The Rockchip imx219 driver is manual-only (no auto_exposure control).
        if auto_exposure:
            return None
        return f"exposure={exposure},gain={gain}"

    # ov5647 (default / legacy behaviour)
    if auto_exposure:
        return "auto_exposure=0,gain_automatic=1"
    return f"auto_exposure=1,gain_automatic=0,exposure={exposure},analogue_gain={gain}"


def _build_flip_ctrl(settings):
    """Returns sensor flip ctrl to disable hardware flip.

    IMX219 (and often OV5647) produce a green cast when flipped in the sensor
    because the Rockchip ISP demosaicing does not track Bayer phase changes.
    Orientation is handled in ``camera.frame_post`` instead.
    """
    return "horizontal_flip=0,vertical_flip=0"


def _apply_ctrl(subdev, model, ctrl):
    """Applies a ctrl string and verifies by reading the values back.

    Some v4l2-ctl builds always exit non-zero on subdevs (the
    VIDIOC_SUBDEV_S_CLIENT_CAP probe fails) even when the controls are applied,
    so success is judged by reading the values back, not the return code.
    """
    result = subprocess.run(
        ["v4l2-ctl", "-d", subdev, f"--set-ctrl={ctrl}"],
        capture_output=True,
        text=True,
    )
    intended = _parse_ctrl(ctrl)
    readback = _get_ctrls(subdev, list(intended)) if intended else {}
    applied = all(readback.get(key) == value for key, value in intended.items())

    if not applied:
        debug.log(
            f"Warning: failed to set sensor controls on {subdev} ({model}): "
            f"intended={intended} readback={readback} "
            f"stderr={result.stderr.strip()}"
        )
        return False

    debug.log(f"Sensor configured: {model} on {subdev}: {ctrl}")
    return True


def _parse_ctrl(ctrl):
    pairs = {}
    for item in ctrl.split(","):
        if "=" in item:
            key, value = item.split("=", 1)
            pairs[key.strip()] = value.strip()
    return pairs


def _get_ctrls(subdev, names):
    """Reads back the given controls as a {name: value} dict (best effort)."""
    result = subprocess.run(
        ["v4l2-ctl", "-d", subdev, "--get-ctrl", ",".join(names)],
        capture_output=True,
        text=True,
    )
    values = {}
    for line in result.stdout.splitlines():
        if ":" in line:
            key, value = line.split(":", 1)
            values[key.strip()] = value.strip()
    return values


def configure_sensor(settings=None):
    """Detects the sensor and applies exposure/gain before opening the camera."""
    settings = settings or {}

    preferred = settings.get("cameraSensorSubdev") or os.environ.get(
        "CAMERA_SENSOR_SUBDEV"
    )
    subdev, model = detect_sensor(preferred)

    if model is None:
        # Unknown sensor: fall back to OV5647 semantics for compatibility.
        debug.log(f"Sensor model not detected on {subdev}; assuming ov5647 controls.")
        model = "ov5647"

    profile = SENSOR_PROFILES[model]

    auto_exposure = str(
        settings.get(
            "cameraAutoExposure",
            os.environ.get("CAMERA_AUTO_EXPOSURE", "0"),
        )
    ).lower() in ("1", "true", "yes")

    exposure = _int_setting(
        settings, "cameraExposure", "CAMERA_EXPOSURE", profile["default_exposure"]
    )
    gain = _int_setting(settings, "cameraGain", "CAMERA_GAIN", profile["default_gain"])
    exposure = _clamp(exposure, *profile["exposure"])
    gain = _clamp(gain, *profile["gain"])

    ok = True

    ctrl = _build_ctrl(model, exposure, gain, auto_exposure)
    if ctrl is None:
        debug.log(
            f"{model} on {subdev}: auto-exposure requested but unsupported; "
            "leaving driver defaults."
        )
    else:
        ok = _apply_ctrl(subdev, model, ctrl) and ok

    flip_ctrl = _build_flip_ctrl(settings)
    ok = _apply_ctrl(subdev, model, flip_ctrl) and ok

    return ok
