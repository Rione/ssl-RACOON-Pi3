"""Capture backend interface.

All backends expose ``read()`` returning ``(ok, frame)`` where ``frame`` is a
BGR numpy array (OpenCV convention), so downstream HSV processing is identical
across boards.
"""

from typing import Protocol, Tuple

import numpy as np


class CaptureBackend(Protocol):
    def read(self) -> Tuple[bool, np.ndarray]:
        ...

    def release(self) -> None:
        ...
