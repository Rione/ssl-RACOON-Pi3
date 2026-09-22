"""TCP server that performs color calibration on demand.

Listens on 127.0.0.1:CALIB_PORT for a request line from the Go API
(/calibballcolor). On "calib", it grabs a frame (sharing the camera with the
main loop via a lock), runs YOLO calibration, saves thresholds, hot-reloads the
detector, and replies with a one-line JSON result.
"""

import json
import socket
import threading

from camera import debug
from camera.settings import CALIB_PORT


class CalibServer(threading.Thread):
    def __init__(self, calibrate_fn, host="127.0.0.1", port=CALIB_PORT):
        super().__init__(daemon=True)
        self._calibrate_fn = calibrate_fn
        self._host = host
        self._port = port

    def run(self):
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        try:
            sock.bind((self._host, self._port))
            sock.listen(1)
        except OSError as e:
            debug.log(f"[calib] Failed to bind calib server on {self._host}:{self._port}: {e}")
            return

        debug.log(f"[calib] Calibration server listening on {self._host}:{self._port}")

        while True:
            try:
                conn, _ = sock.accept()
            except OSError as e:
                debug.log(f"[calib] accept error: {e}")
                continue

            with conn:
                self._handle_request(conn)

    def _handle_request(self, conn):
        try:
            data = conn.recv(64)
        except OSError as e:
            debug.log(f"[calib] recv error: {e}")
            return

        request = data.decode("utf-8", errors="ignore").strip()
        if request != "calib":
            self._send(conn, {"ok": False, "error": f"unknown request: {request}"})
            return

        try:
            result = self._calibrate_fn()
        except Exception as e:
            if debug.enabled():
                import traceback

                traceback.print_exc()
            result = {"ok": False, "error": f"calibration failed: {e}"}

        self._send(conn, result)

    def _send(self, conn, result):
        try:
            payload = json.dumps(result) + "\n"
            conn.sendall(payload.encode("utf-8"))
        except OSError as e:
            debug.log(f"[calib] send error: {e}")
