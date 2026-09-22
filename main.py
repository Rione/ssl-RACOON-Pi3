"""Backward-compatible entry point.

The camera implementation now lives in the ``camera`` package. This wrapper is
kept so existing ``python3 main.py`` invocations still work; prefer
``python3 -m camera``.
"""

from camera.main import main

if __name__ == "__main__":
    main()
