"""HSV color + contour based ball detection.

Moved from the original main.py. ``ImageProcessor`` and ``BallDetector`` support
hot-reloading their thresholds via ``update_settings`` so calibration can take
effect without restarting the process.
"""

import cv2
import numpy as np

from camera import debug


class ImageProcessor:
    def __init__(self, settings=None):
        if settings is None:
            settings = {}

        self._minThreshold = settings.get(
            "minThreshold", np.array([1, 120, 100], dtype=np.uint8)
        )
        self._maxThreshold = settings.get(
            "maxThreshold", np.array([15, 255, 255], dtype=np.uint8)
        )
        self._ksize = tuple(settings.get("gaussianKernelSize", (5, 5)))
        self._sigmaX = settings.get("gaussianSigmaX", 0)
        self._shape = cv2.MORPH_RECT
        self._size = tuple(settings.get("morphKernelSize", (3, 3)))
        # Closing bridges the gaps that printed text/logos punch into the ball
        # mask so the silhouette stays a single connected blob. Larger than the
        # opening kernel because it has to span the stroke width of the text.
        self._closeSize = self._normalize_kernel(
            settings.get("morphCloseKernelSize", (9, 9))
        )
        # After closing, any holes fully enclosed by the ball (e.g. dark numbers
        # in the middle) are filled so the blob is solid. Cheap on the small ROI.
        self._fillHoles = bool(settings.get("fillHoles", True))
        self._open_kernel = cv2.getStructuringElement(self._shape, self._size)
        self._close_kernel = (
            cv2.getStructuringElement(self._shape, self._closeSize)
            if self._closeSize
            else None
        )

    @staticmethod
    def _normalize_kernel(value):
        """Returns a (w, h) kernel tuple, or None to disable the step."""
        if value is None:
            return None
        if isinstance(value, int):
            value = (value, value)
        size = tuple(int(v) for v in value)
        if len(size) != 2 or size[0] <= 1 or size[1] <= 1:
            return None
        return size

    def update_thresholds(self, min_threshold, max_threshold):
        self._minThreshold = min_threshold
        self._maxThreshold = max_threshold

    def extractColors(self, frame):
        filtered = self._filterFrame(frame)
        hsv = cv2.cvtColor(filtered, cv2.COLOR_BGR2HSV)
        hsv = self._equalizeHist(hsv)
        mask = cv2.inRange(hsv, self._minThreshold, self._maxThreshold)
        mask = self._applyMorphologicalTransformations(mask)
        return mask

    def _filterFrame(self, frame):
        return cv2.GaussianBlur(frame, self._ksize, self._sigmaX)

    def _equalizeHist(self, hsv):
        h, s, v = cv2.split(hsv)
        clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
        v_equalized = clahe.apply(v)
        return cv2.merge((h, s, v_equalized))

    def _applyMorphologicalTransformations(self, mask):
        # Open first to drop speckle noise, then close to seal text-induced gaps.
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, self._open_kernel)
        if self._close_kernel is not None:
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, self._close_kernel)
        if self._fillHoles:
            mask = self._fill_internal_holes(mask)
        return mask

    @staticmethod
    def _fill_internal_holes(mask):
        """Fills holes fully enclosed by a blob without merging separate blobs.

        Uses the external contours (which already ignore interior holes) and
        redraws them filled. One findContours + draw pass, so it stays cheap on
        the ball-sized ROI while making the silhouette solid for a stable
        minEnclosingCircle fit and a clean mask preview.
        """
        contours, _ = cv2.findContours(
            mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )
        if contours:
            cv2.drawContours(mask, contours, -1, 255, thickness=cv2.FILLED)
        return mask


class BallDetector:
    def __init__(self, settings=None):
        if settings is None:
            settings = {}

        self.imageProcessor = ImageProcessor(settings)
        self._radius = int(settings.get("ballDetectRadius", 150))
        self._circularityThreshold = float(settings.get("circularityThreshold", 0.2))
        self._minContourArea = float(settings.get("minContourArea", 100))
        self._previousCenter = None
        self._missCount = 0
        self._fullFrameSearch = False
        self._fullFrameAfterMisses = int(settings.get("fullFrameAfterMisses", 3))

    def update_settings(self, settings):
        """Hot-reloads thresholds after a calibration run."""
        self.imageProcessor.update_thresholds(
            settings.get("minThreshold", self.imageProcessor._minThreshold),
            settings.get("maxThreshold", self.imageProcessor._maxThreshold),
        )
        if "ballDetectRadius" in settings:
            self._radius = int(settings["ballDetectRadius"])
        if "circularityThreshold" in settings:
            self._circularityThreshold = float(settings["circularityThreshold"])
        if "fullFrameAfterMisses" in settings:
            self._fullFrameAfterMisses = int(settings["fullFrameAfterMisses"])
        self._previousCenter = None
        self._missCount = 0
        self._fullFrameSearch = False

    def _register_miss(self):
        self._missCount += 1
        if self._missCount >= self._fullFrameAfterMisses:
            self._fullFrameSearch = True

    def _register_hit(self, center):
        self._missCount = 0
        self._fullFrameSearch = False
        self._previousCenter = center

    def detect(self, frame):
        search_center = None if self._fullFrameSearch else self._previousCenter
        roi, offset, vertices = self._focus(frame, search_center)
        if roi.size == 0:
            debug.log("Warning: ROI is empty.")
            self._register_miss()
            self._previousCenter = None
            return None, None, None, None

        mask = self.imageProcessor.extractColors(roi)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        valid_contours = [
            cnt for cnt in contours if cv2.contourArea(cnt) > self._minContourArea
        ]

        if not valid_contours:
            self._register_miss()
            return None, None, vertices, None

        bestContour = max(valid_contours, key=cv2.contourArea)

        if self._isCircular(bestContour):
            (x, y), radius = cv2.minEnclosingCircle(bestContour)
            center = (int(x + offset[0]), int(y + offset[1]))
            circleContour = self._createCircleContour(
                x + offset[0], y + offset[1], radius
            )
            self._register_hit(center)
            diameter = int(radius * 2)
            distance = 320 * 40 / diameter if diameter > 0 else None
            return center, circleContour, vertices, distance

        self._register_miss()
        return None, None, vertices, None

    def _isCircular(self, contour):
        perimeter = cv2.arcLength(contour, True)
        area = cv2.contourArea(contour)
        if perimeter == 0 or area == 0:
            return False
        circularity = (4 * np.pi * area) / (perimeter**2)
        return circularity > self._circularityThreshold

    def _focus(self, frame, center):
        height, width = frame.shape[:2]
        radius = self._radius

        if center is not None:
            cx, cy = center
            xMin = max(0, cx - radius)
            yMin = max(0, cy - radius)
            xMax = min(width, cx + radius)
            yMax = min(height, cy + radius)
        else:
            xMin, yMin, xMax, yMax = 0, 0, width, height

        xMin, yMin, xMax, yMax = int(xMin), int(yMin), int(xMax), int(yMax)

        if yMin >= yMax or xMin >= xMax:
            debug.log(
                f"Warning: Invalid ROI calculated: ({xMin},{yMin}) to ({xMax},{yMax}). Using full frame."
            )
            xMin, yMin, xMax, yMax = 0, 0, width, height

        roi = frame[yMin:yMax, xMin:xMax]
        offset = (xMin, yMin)
        vertices = (xMin, yMin, xMax, yMax)
        return roi, offset, vertices

    def _createCircleContour(self, centerX, centerY, radius, num_points=36):
        angles = np.linspace(0, 2 * np.pi, num_points, endpoint=False)
        contour_points = np.array(
            [
                [centerX + radius * np.cos(ang), centerY + radius * np.sin(ang)]
                for ang in angles
            ],
            dtype=np.int32,
        )
        return contour_points.reshape((-1, 1, 2))

    @staticmethod
    def encode_jpeg_b64(frame, quality=75, max_width=480):
        import base64

        if frame is None or frame.size == 0:
            return None
        h, w = frame.shape[:2]
        if w > max_width:
            scale = max_width / w
            frame = cv2.resize(frame, None, fx=scale, fy=scale, interpolation=cv2.INTER_AREA)
        ok, encoded = cv2.imencode(".jpg", frame, [int(cv2.IMWRITE_JPEG_QUALITY), quality])
        if not ok:
            return None
        return base64.b64encode(encoded.tobytes()).decode("utf-8")

    def build_mask_overlay(self, frame, max_width=480):
        """Returns a BGR frame highlighting pixels matching the current HSV mask."""
        if frame is None or frame.size == 0:
            return frame
        work = frame
        h, w = work.shape[:2]
        if w > max_width:
            scale = max_width / w
            work = cv2.resize(work, None, fx=scale, fy=scale, interpolation=cv2.INTER_AREA)

        mask = self.imageProcessor.extractColors(work)
        overlay = work.copy()
        detected = mask > 0
        overlay[detected] = (
            overlay[detected].astype(np.float32) * 0.4 + np.array([0, 255, 0], dtype=np.float32) * 0.6
        ).astype(np.uint8)
        return overlay


class Visualizer:
    def __init__(self, radius=5, windowName="Frame"):
        self._radius = radius
        self._windowName = windowName

    def draw(self, frame, center, circleContour, vertices):
        if vertices:
            cv2.rectangle(
                frame,
                (vertices[0], vertices[1]),
                (vertices[2], vertices[3]),
                (0, 0, 255),
                1,
            )

        if circleContour is not None:
            cv2.drawContours(frame, [circleContour], -1, (255, 0, 0), 2)

        if center is not None:
            cv2.circle(frame, center, self._radius, (0, 255, 0), -1)

        return frame

    def destroy(self):
        pass
