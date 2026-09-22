"""Camera debug logging.

Enabled when RACOON_CAMERA_DEBUG is set (by Go -dc flag) or when
threshold.json contains "cameraDebug": true.
"""

import os

_debug = None


def _env_enabled():
    return os.environ.get("RACOON_CAMERA_DEBUG", "").strip().lower() in (
        "1",
        "true",
        "yes",
        "on",
    )


def configure(settings=None):
    """Applies debug settings from the environment and optional settings dict."""
    global _debug
    if _env_enabled():
        _debug = True
        return
    if settings is not None and "cameraDebug" in settings:
        _debug = bool(settings["cameraDebug"])
        return
    if _debug is None:
        _debug = False


def enabled():
    if _debug is None:
        configure()
    return _debug


def log(*args, **kwargs):
    if enabled():
        print(*args, **kwargs)
