"""YOLO-assisted HSV color calibration.

Runs the YOLO model (from the ssl-YOLO-Detection submodule) on a single frame,
locates the ball bounding box, samples HSV at the center plus four points
(up/down/left/right), and derives min/max HSV thresholds with a safety margin.

YOLO is imported lazily so the heavy ultralytics dependency is only loaded when
calibration is actually requested, keeping the normal detection loop light.
"""

import base64
import os

import cv2
import numpy as np

from camera import debug
from camera.detect.color import ImageProcessor

_PACKAGE_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
_MODEL_DIR = os.path.join(_PACKAGE_DIR, "yolo")

# Fraction of the ball radius used to place the up/down/left/right sample points.
# Kept well inside the bbox so background/edge pixels are not sampled.
SAMPLE_OFFSET_RATIO = 0.35

# Margins added around the sampled HSV range to tolerate lighting variation.
H_MARGIN = 7
S_MARGIN = 35
V_MARGIN = 35

_model = None


def _find_model():
    for name in ("last.pt", "best.pt"):
        path = os.path.join(_MODEL_DIR, name)
        if os.path.exists(path):
            return path
    return None


def _load_model():
    """Loads (and caches) the YOLO model. Returns None if unavailable."""
    global _model
    if _model is not None:
        return _model

    model_path = _find_model()
    if model_path is None:
        debug.log(f"[calib] YOLO model not found in {_MODEL_DIR}")
        return None

    try:
        from ultralytics import YOLO
    except ImportError as e:
        debug.log(f"[calib] ultralytics not installed: {e}")
        return None

    debug.log(f"[calib] Loading YOLO model: {model_path}")
    _model = YOLO(model_path)
    return _model


def _best_box(result):
    """Returns (x1, y1, x2, y2) of the highest-confidence detection, or None."""
    boxes = result.boxes
    if boxes is None or len(boxes) == 0:
        return None

    best = None
    best_conf = -1.0
    for box in boxes:
        conf = float(box.conf[0])
        if conf > best_conf:
            best_conf = conf
            best = box
    if best is None:
        return None

    x1, y1, x2, y2 = (int(v) for v in best.xyxy[0])
    return x1, y1, x2, y2, best_conf


def _sample_points(bbox, frame_shape):
    """Computes the 5 sample points within the bbox, clamped to the frame."""
    x1, y1, x2, y2 = bbox
    h, w = frame_shape[:2]
    cx = (x1 + x2) // 2
    cy = (y1 + y2) // 2
    r = min(x2 - x1, y2 - y1) / 2.0
    off = int(r * SAMPLE_OFFSET_RATIO)

    points = [
        (cx, cy),
        (cx, cy - off),
        (cx, cy + off),
        (cx - off, cy),
        (cx + off, cy),
    ]

    clamped = []
    for px, py in points:
        px = max(0, min(w - 1, px))
        py = max(0, min(h - 1, py))
        clamped.append((px, py))
    return clamped


def _encode_preview(frame, bbox, points, quality=80):
    preview = frame.copy()
    x1, y1, x2, y2 = bbox
    cv2.rectangle(preview, (x1, y1), (x2, y2), (0, 255, 0), 2)
    for px, py in points:
        cv2.circle(preview, (px, py), 3, (0, 0, 255), -1)

    ok, encoded = cv2.imencode(".jpg", preview, [int(cv2.IMWRITE_JPEG_QUALITY), quality])
    if not ok:
        return None
    return base64.b64encode(encoded.tobytes()).decode("utf-8")


def calibrate(frame, settings=None, conf=0.25):
    """Runs YOLO calibration on a BGR frame.

    Returns a dict: on success ``{"ok": True, "minThreshold", "maxThreshold",
    "ballDetectRadius", "bbox", "samplePoints", "previewFrame"}``; on failure
    ``{"ok": False, "error": ...}``.
    """
    if settings is None:
        settings = {}

    if frame is None or frame.size == 0:
        return {"ok": False, "error": "empty frame"}

    model = _load_model()
    if model is None:
        return {"ok": False, "error": "YOLO model unavailable"}

    frame = np.ascontiguousarray(frame)

    def _run_yolo(infer_conf):
        return model(frame, conf=infer_conf, imgsz=640, verbose=False)

    results = _run_yolo(conf)
    if not results or _best_box(results[0]) is None:
        # Retry with a lower threshold (small ball / wide FOV / colour cast).
        results = _run_yolo(max(0.08, conf * 0.5))
    if not results or _best_box(results[0]) is None:
        # Retry on a 180°-rotated copy when camera orientation is wrong.
        rotated = cv2.rotate(frame, cv2.ROTATE_180)
        results = model(rotated, conf=max(0.08, conf * 0.5), imgsz=640, verbose=False)
        if results and _best_box(results[0]) is not None:
            frame = rotated
    if not results:
        return {"ok": False, "error": "ball not detected"}

    box = _best_box(results[0])
    if box is None:
        return {"ok": False, "error": "ball not detected"}

    x1, y1, x2, y2, detection_conf = box
    bbox = (x1, y1, x2, y2)
    if x2 <= x1 or y2 <= y1:
        return {"ok": False, "error": "invalid bounding box"}

    points = _sample_points(bbox, frame.shape)

    # Preprocess identically to the runtime HSV pipeline so calibrated
    # thresholds match what the detector actually sees.
    processor = ImageProcessor(settings)
    blurred = processor._filterFrame(frame)
    hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
    hsv = processor._equalizeHist(hsv)

    samples = np.array([hsv[py, px] for px, py in points], dtype=np.int32)
    h_min, s_min, v_min = samples.min(axis=0)
    h_max, s_max, v_max = samples.max(axis=0)

    min_threshold = np.array(
        [
            max(0, h_min - H_MARGIN),
            max(0, s_min - S_MARGIN),
            max(0, v_min - V_MARGIN),
        ],
        dtype=np.uint8,
    )
    max_threshold = np.array(
        [
            min(179, h_max + H_MARGIN),
            min(255, s_max + S_MARGIN),
            min(255, v_max + V_MARGIN),
        ],
        dtype=np.uint8,
    )

    # Estimate ROI radius from the bbox, keeping at least the existing value.
    ball_px_radius = int(min(x2 - x1, y2 - y1) / 2.0)
    existing_radius = int(settings.get("ballDetectRadius", 150))
    ball_detect_radius = max(existing_radius, int(ball_px_radius * 1.5))

    return {
        "ok": True,
        "minThreshold": min_threshold,
        "maxThreshold": max_threshold,
        "ballDetectRadius": ball_detect_radius,
        "bbox": [int(x1), int(y1), int(x2), int(y2)],
        "confidence": round(float(detection_conf), 3),
        "samplePoints": [[int(px), int(py)] for px, py in points],
        "previewFrame": _encode_preview(frame, bbox, points),
    }
