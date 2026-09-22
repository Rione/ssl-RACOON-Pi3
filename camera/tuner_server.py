"""TCP server for live HSV threshold tuning (preview + hot-reload)."""

import json
import socket
import threading

import numpy as np

from camera import debug
from camera.settings import TUNER_PORT


def _json_default(obj):
    if isinstance(obj, (np.integer,)):
        return int(obj)
    if isinstance(obj, (np.floating,)):
        return float(obj)
    if isinstance(obj, np.ndarray):
        return obj.tolist()
    if isinstance(obj, (np.bool_,)):
        return bool(obj)
    raise TypeError(f"Object of type {type(obj).__name__} is not JSON serializable")


class TunerServer(threading.Thread):
    def __init__(self, preview_fn, apply_fn, relax_fn, host="127.0.0.1", port=TUNER_PORT):
        super().__init__(daemon=True)
        self._preview_fn = preview_fn
        self._apply_fn = apply_fn
        self._relax_fn = relax_fn
        self._host = host
        self._port = port
        self._ready = threading.Event()

    def wait_ready(self, timeout=5.0):
        return self._ready.wait(timeout)

    def run(self):
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        try:
            sock.bind((self._host, self._port))
            sock.listen(4)
        except OSError as e:
            debug.log(f"[tuner] Failed to bind on {self._host}:{self._port}: {e}")
            return

        self._ready.set()
        debug.log(f"[tuner] Threshold tuner listening on {self._host}:{self._port}")

        while True:
            try:
                conn, _ = sock.accept()
            except OSError as e:
                debug.log(f"[tuner] accept error: {e}")
                continue

            with conn:
                self._handle_request(conn)

    def _handle_request(self, conn):
        try:
            data = conn.recv(512)
        except OSError as e:
            debug.log(f"[tuner] recv error: {e}")
            return

        request = data.decode("utf-8", errors="ignore").strip()
        try:
            if request == "preview":
                result = self._preview_fn()
            elif request.startswith("set|"):
                parts = request.split("|")
                if len(parts) != 6:
                    result = {"ok": False, "error": "bad set format"}
                else:
                    result = self._apply_fn(
                        parts[1],
                        parts[2],
                        int(parts[3]),
                        float(parts[4]),
                        parts[5] == "1",
                    )
            elif request.startswith("relax|"):
                save = request.split("|", 1)[1] == "1"
                result = self._relax_fn(save)
            else:
                result = {"ok": False, "error": f"unknown request: {request}"}
        except Exception as e:
            if debug.enabled():
                import traceback

                traceback.print_exc()
            result = {"ok": False, "error": str(e)}

        self._send(conn, result)

    def _send(self, conn, result):
        try:
            payload = json.dumps(result, default=_json_default) + "\n"
            conn.sendall(payload.encode("utf-8"))
        except Exception as e:
            debug.log(f"[tuner] send error: {e}")
            try:
                fallback = json.dumps({"ok": False, "error": f"send failed: {e}"}) + "\n"
                conn.sendall(fallback.encode("utf-8"))
            except OSError:
                pass
