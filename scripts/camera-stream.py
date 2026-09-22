#!/usr/bin/env python3
"""Low-latency MJPEG HTTP stream for Rock 5A.

The v4l2-ctl | ffmpeg pipe queues many seconds of raw frames when MJPEG
encoding cannot keep up, which looks like ~1 FPS and large lag. This server
captures with OpenCV (buffer size 1) and always serves the latest frame.
"""

import argparse
import os
import threading
import time

import cv2
import numpy as np
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

from camera.frame_post import postprocess_frame
from camera.sensor import configure_sensor, detect_sensor

latest_frame = None
frame_seq = 0
frame_lock = threading.Lock()
_gamma_lut = None


def build_gamma_lut(gamma):
    inv_gamma = 1.0 / max(gamma, 0.1)
    return np.array(
        [((i / 255.0) ** inv_gamma) * 255 for i in range(256)],
        dtype=np.uint8,
    )


def enhance_frame(frame, gamma, brightness):
    global _gamma_lut

    if gamma != 1.0:
        if _gamma_lut is None:
            _gamma_lut = build_gamma_lut(gamma)
        frame = cv2.LUT(frame, _gamma_lut)

    if brightness != 0:
        frame = cv2.convertScaleAbs(frame, alpha=1.0, beta=brightness)

    return frame


def capture_loop(device, width, height, fps, gamma, brightness, settings, sensor_model):
    global latest_frame, frame_seq

    cap = cv2.VideoCapture(device)
    if not cap.isOpened():
        raise RuntimeError(f"Cannot open camera device {device}")

    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
    if fps > 0:
        cap.set(cv2.CAP_PROP_FPS, fps)

    print(
        f"Capture started on /dev/video{device}: target {width}x{height} @ {fps}fps"
    )

    while True:
        ret, frame = cap.read()
        if not ret:
            time.sleep(0.02)
            continue

        frame = postprocess_frame(frame, settings, sensor_model)
        frame = enhance_frame(frame, gamma, brightness)

        with frame_lock:
            latest_frame = frame
            frame_seq += 1


class StreamHandler(BaseHTTPRequestHandler):
    quality = 65
    min_interval = 1.0 / 15.0

    def do_GET(self):
        if self.path not in ("/", "/stream.mjpg", "/video"):
            self.send_error(404)
            return

        self.send_response(200)
        self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
        self.send_header("Cache-Control", "no-cache, no-store, must-revalidate")
        self.send_header("Pragma", "no-cache")
        self.send_header("Connection", "close")
        self.end_headers()

        last_sent_seq = -1
        encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), self.quality]

        try:
            while True:
                with frame_lock:
                    seq = frame_seq
                    frame = latest_frame

                if frame is None or seq == last_sent_seq:
                    time.sleep(0.005)
                    continue

                ok, buf = cv2.imencode(".jpg", frame, encode_param)
                if not ok:
                    continue

                last_sent_seq = seq
                self.wfile.write(b"--frame\r\nContent-Type: image/jpeg\r\n\r\n")
                self.wfile.write(buf.tobytes())
                self.wfile.write(b"\r\n")
                self.wfile.flush()
                time.sleep(self.min_interval)
        except (BrokenPipeError, ConnectionResetError, OSError):
            pass

    def log_message(self, fmt, *args):
        pass


def parse_args():
    parser = argparse.ArgumentParser(description="MJPEG HTTP camera stream")
    parser.add_argument(
        "--device",
        type=int,
        default=int(os.environ.get("CAMERA_DEVICE", "11")),
        help="V4L2 device index (default: 11 = /dev/video11)",
    )
    parser.add_argument(
        "--width",
        type=int,
        default=int(os.environ.get("CAMERA_WIDTH", "640")),
    )
    parser.add_argument(
        "--height",
        type=int,
        default=int(os.environ.get("CAMERA_HEIGHT", "480")),
    )
    parser.add_argument(
        "--fps",
        type=int,
        default=int(os.environ.get("CAMERA_FPS", "15")),
        help="Target stream rate (camera may deliver ~10-15 FPS on Rock 5A)",
    )
    parser.add_argument(
        "--port",
        type=int,
        default=int(os.environ.get("STREAM_PORT", "8080")),
    )
    parser.add_argument(
        "--quality",
        type=int,
        default=int(os.environ.get("STREAM_QUALITY", "65")),
        help="JPEG quality 1-100 (lower = faster, smaller)",
    )
    parser.add_argument(
        "--sensor-subdev",
        default=os.environ.get("CAMERA_SENSOR_SUBDEV", "/dev/v4l-subdev2"),
        help="V4L2 subdev for the camera sensor (OV5647 on Rock 5A)",
    )
    parser.add_argument(
        "--exposure",
        type=int,
        default=int(os.environ.get("CAMERA_EXPOSURE", "900")),
        help="Manual sensor exposure (4-1964)",
    )
    parser.add_argument(
        "--gain",
        type=int,
        default=int(os.environ.get("CAMERA_GAIN", "80")),
        help="Manual sensor analogue gain (16-1023)",
    )
    parser.add_argument(
        "--auto-exposure",
        action="store_true",
        default=os.environ.get("CAMERA_AUTO_EXPOSURE", "").lower() in ("1", "true", "yes"),
        help="Use sensor auto exposure instead of manual gain/exposure",
    )
    parser.add_argument(
        "--gamma",
        type=float,
        default=float(os.environ.get("STREAM_GAMMA", "1.1")),
        help="Software gamma correction (>1 brightens mid-tones)",
    )
    parser.add_argument(
        "--brightness",
        type=int,
        default=int(os.environ.get("STREAM_BRIGHTNESS", "0")),
        help="Software brightness offset (-255 to 255)",
    )
    return parser.parse_args()


def main():
    args = parse_args()

    # Re-use threshold.json for flip/colour settings when present.
    try:
        from camera.settings import load_settings

        stream_settings = load_settings()
    except Exception:
        stream_settings = {}

    stream_settings.update(
        {
            "cameraSensorSubdev": args.sensor_subdev,
            "cameraExposure": args.exposure,
            "cameraGain": args.gain,
            "cameraAutoExposure": args.auto_exposure,
        }
    )

    configure_sensor(stream_settings)
    _, sensor_model = detect_sensor(args.sensor_subdev)

    threading.Thread(
        target=capture_loop,
        args=(
            args.device,
            args.width,
            args.height,
            args.fps,
            max(0.1, args.gamma),
            max(-255, min(args.brightness, 255)),
            stream_settings,
            sensor_model,
        ),
        daemon=True,
    ).start()

    deadline = time.time() + 10.0
    while frame_seq == 0 and time.time() < deadline:
        time.sleep(0.05)
    if frame_seq == 0:
        raise RuntimeError("Timed out waiting for the first camera frame")

    StreamHandler.quality = max(1, min(args.quality, 100))
    StreamHandler.min_interval = 1.0 / max(1, args.fps)

    server = ThreadingHTTPServer(("0.0.0.0", args.port), StreamHandler)
    print(f"Stream ready: http://0.0.0.0:{args.port}/stream.mjpg")
    server.serve_forever()


if __name__ == "__main__":
    main()
