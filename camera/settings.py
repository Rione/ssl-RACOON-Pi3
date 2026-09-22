"""Settings loading/saving for the camera package.

The threshold configuration is shared with the Go side, which reads and writes
``threshold.json`` (see internal/mw/mw.go). HSV thresholds are stored as
comma-separated strings (e.g. "1,115,90") in that file, so they are parsed into
numpy arrays here.
"""

import json
import os

import numpy as np

from camera import debug

# UDP port the camera publishes detection results to (consumed by Go receive).
UDP_CAMERA_PORT = 31133
# TCP port the calibration server listens on (driven by Go /calibballcolor).
CALIB_PORT = 31134
# TCP port for live threshold preview/tuning (Go /color-tuner UI).
TUNER_PORT = 31135

CONFIG_FILENAME = "threshold.json"

_PACKAGE_DIR = os.path.dirname(os.path.abspath(__file__))
_REPO_ROOT = os.path.dirname(_PACKAGE_DIR)


def config_path():
    """Returns the threshold.json path, preferring the current working dir.

    The Go process launches the camera with its working directory set to the
    binary location, where it also keeps threshold.json. Falling back to the
    repository root keeps standalone runs working.
    """
    cwd_path = os.path.join(os.getcwd(), CONFIG_FILENAME)
    if os.path.exists(cwd_path):
        return cwd_path

    repo_path = os.path.join(_REPO_ROOT, CONFIG_FILENAME)
    if os.path.exists(repo_path):
        return repo_path

    # Neither exists yet: default to cwd so saves land next to the Go binary.
    return cwd_path


def parse_threshold(value, default):
    """Parses an HSV threshold into a uint8 numpy array.

    Accepts the Go comma-separated string form ("1,115,90"), a JSON list, or a
    numpy array. Returns ``default`` when the value is missing or invalid.
    """
    if value is None:
        return default
    if isinstance(value, np.ndarray):
        return value.astype(np.uint8)
    if isinstance(value, str):
        try:
            parts = [int(x.strip()) for x in value.split(",") if x.strip() != ""]
        except ValueError:
            return default
        if len(parts) != 3:
            return default
        return np.array(parts, dtype=np.uint8)
    if isinstance(value, (list, tuple)):
        if len(value) != 3:
            return default
        return np.array(value, dtype=np.uint8)
    return default


def load_settings():
    """Loads settings from threshold.json, returning a dict with parsed values."""
    path = config_path()
    settings = {}
    load_error = None

    if os.path.exists(path):
        try:
            with open(path, "r") as f:
                settings = json.load(f)
        except json.JSONDecodeError as e:
            load_error = f"Error decoding JSON from {path}: {e}"
            settings = {}
        except Exception as e:
            load_error = f"An unexpected error occurred while reading {path}: {e}"
            settings = {}

    settings["minThreshold"] = parse_threshold(
        settings.get("minThreshold"), np.array([1, 120, 100], dtype=np.uint8)
    )
    settings["maxThreshold"] = parse_threshold(
        settings.get("maxThreshold"), np.array([15, 255, 255], dtype=np.uint8)
    )
    if "ballDetectRadius" in settings:
        settings["ballDetectRadius"] = int(settings["ballDetectRadius"])
    if "circularityThreshold" in settings:
        settings["circularityThreshold"] = float(settings["circularityThreshold"])

    debug.configure(settings)
    if os.path.exists(path):
        debug.log(f"Reading JSON settings file: {path}")
        if load_error is None:
            debug.log("JSON settings loaded successfully.")
        else:
            debug.log(load_error)
    else:
        debug.log(f"Warning: Settings file '{path}' not found. Using default values.")

    return settings


def threshold_to_string(arr):
    """Formats a 3-element HSV array as the Go comma-separated string form."""
    return ",".join(str(int(v)) for v in arr)


def save_thresholds(min_threshold, max_threshold, ball_detect_radius, circularity_threshold):
    """Writes the threshold config back to threshold.json (Go-compatible form).

    ``min_threshold`` / ``max_threshold`` may be numpy arrays or strings. Other
    existing keys in the file are preserved.
    """
    path = config_path()

    data = {}
    if os.path.exists(path):
        try:
            with open(path, "r") as f:
                data = json.load(f)
        except Exception:
            data = {}

    if not isinstance(min_threshold, str):
        min_threshold = threshold_to_string(min_threshold)
    if not isinstance(max_threshold, str):
        max_threshold = threshold_to_string(max_threshold)

    data["minThreshold"] = min_threshold
    data["maxThreshold"] = max_threshold
    data["ballDetectRadius"] = int(ball_detect_radius)
    data["circularityThreshold"] = float(circularity_threshold)

    with open(path, "w") as f:
        json.dump(data, f)

    return data
