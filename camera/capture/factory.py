"""Selects the capture backend based on the RACOON_BOARD environment variable.

The Go process injects ``RACOON_BOARD`` (pi4 or rock5a) via build-tag specific
code. Defaults to pi4 to preserve the previous behaviour when unset.
"""

import os

from camera import debug


def create_capture(settings=None):
    board = os.environ.get("RACOON_BOARD", "pi4").strip().lower()

    if board == "rock5a":
        from camera.capture.rock5a import Rock5aCapture

        debug.log("Selected capture backend: rock5a (V4L2)")
        return Rock5aCapture(settings)

    # Default / pi4
    from camera.capture.pi4 import Pi4Capture

    debug.log("Selected capture backend: pi4 (Picamera2)")
    return Pi4Capture(settings)
