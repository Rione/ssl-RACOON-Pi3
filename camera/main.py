"""Camera main loop with on-demand YOLO color calibration.

Normal operation runs only the lightweight HSV + contour detector and publishes
results over UDP. A background calibration server shares the same camera and,
when triggered, runs YOLO once to recompute HSV thresholds.
"""

import threading
import time

import cv2

from camera import debug
from camera.calib_server import CalibServer
from camera.capture.factory import create_capture
from camera.detect.calib import calibrate
from camera.detect.color import BallDetector, Visualizer
from camera.settings import load_settings, save_thresholds, threshold_to_string
from camera.threshold_utils import arrays_to_strings, relax_arrays, strings_to_arrays
from camera.transport.encoder import Encoder, NO_BALL_COORD
from camera.transport.udp_client import UDPClient
from camera.tuner_server import TunerServer


class CameraContext:
    """Shared, lock-guarded state between the main loop and calib server."""

    def __init__(self, capture, detector, settings):
        self.capture = capture
        self.detector = detector
        self.settings = settings
        self.lock = threading.Lock()

    def read(self):
        with self.lock:
            return self.capture.read()

    def capture_detect(self):
        """Thread-safe capture + detect for the main loop and tuner preview."""
        with self.lock:
            ret, frame = self.capture.read()
            if not ret or frame is None:
                return None, None, None, None, None
            frame = frame.copy()
            center, circle_contour, vertices, distance = self.detector.detect(frame)
            return frame, center, circle_contour, vertices, distance

    def run_calibration(self):
        """Captures a frame, calibrates, persists, and hot-reloads the detector.

        Returns a JSON-serializable dict for the calib server to send back.
        """
        with self.lock:
            ret, frame = self.capture.read()

        if not ret or frame is None:
            return {"ok": False, "error": "failed to capture frame"}

        result = calibrate(frame, self.settings)
        if not result.get("ok"):
            return {"ok": False, "error": result.get("error", "calibration failed")}

        min_threshold = result["minThreshold"]
        max_threshold = result["maxThreshold"]
        ball_detect_radius = result["ballDetectRadius"]
        circularity_threshold = float(self.settings.get("circularityThreshold", 0.2))

        save_thresholds(
            min_threshold, max_threshold, ball_detect_radius, circularity_threshold
        )

        # Update parsed settings and hot-reload the detector under the lock.
        self.settings["minThreshold"] = min_threshold
        self.settings["maxThreshold"] = max_threshold
        self.settings["ballDetectRadius"] = ball_detect_radius
        with self.lock:
            self.detector.update_settings(self.settings)

        return {
            "ok": True,
            "minThreshold": threshold_to_string(min_threshold),
            "maxThreshold": threshold_to_string(max_threshold),
            "ballDetectRadius": int(ball_detect_radius),
            "bbox": result["bbox"],
            "confidence": result.get("confidence"),
            "samplePoints": result["samplePoints"],
            "previewFrame": result.get("previewFrame"),
        }

    def run_preview(self):
        frame, center, circle_contour, vertices, _distance = self.capture_detect()
        if frame is None:
            return {"ok": False, "error": "failed to capture frame"}

        overlay = self.detector.build_mask_overlay(frame)
        annotated = Visualizer().draw(frame, center, circle_contour, vertices)

        frame_h, frame_w = frame.shape[:2]
        is_ball = center is not None
        if is_ball:
            graph_x, graph_y = Encoder.pixel_to_graph(
                center[0], center[1], frame_w, frame_h
            )
        else:
            graph_x, graph_y = NO_BALL_COORD, NO_BALL_COORD

        min_str, max_str = arrays_to_strings(
            self.settings["minThreshold"],
            self.settings["maxThreshold"],
        )

        return {
            "ok": True,
            "isball": bool(is_ball),
            "x": float(graph_x),
            "y": float(graph_y),
            "minThreshold": min_str,
            "maxThreshold": max_str,
            "ballDetectRadius": int(self.settings.get("ballDetectRadius", 150)),
            "circularityThreshold": float(self.settings.get("circularityThreshold", 0.2)),
            "cameraFrame": BallDetector.encode_jpeg_b64(annotated),
            "maskFrame": BallDetector.encode_jpeg_b64(overlay),
        }

    def apply_thresholds(self, min_str, max_str, ball_detect_radius, circularity_threshold, save):
        min_t, max_t = strings_to_arrays(min_str, max_str, self.settings)

        self.settings["minThreshold"] = min_t
        self.settings["maxThreshold"] = max_t
        self.settings["ballDetectRadius"] = int(ball_detect_radius)
        self.settings["circularityThreshold"] = float(circularity_threshold)

        with self.lock:
            self.detector.update_settings(self.settings)

        if save:
            save_thresholds(
                min_t,
                max_t,
                ball_detect_radius,
                circularity_threshold,
            )

        min_out, max_out = arrays_to_strings(min_t, max_t)
        return {
            "ok": True,
            "saved": bool(save),
            "minThreshold": min_out,
            "maxThreshold": max_out,
            "ballDetectRadius": int(ball_detect_radius),
            "circularityThreshold": float(circularity_threshold),
        }

    def relax_thresholds(self, save=False):
        min_t, max_t = relax_arrays(
            self.settings["minThreshold"],
            self.settings["maxThreshold"],
        )
        min_str, max_str = arrays_to_strings(min_t, max_t)
        return self.apply_thresholds(
            min_str,
            max_str,
            int(self.settings.get("ballDetectRadius", 150)),
            float(self.settings.get("circularityThreshold", 0.2)),
            save,
        )


def main():
    settings = load_settings()

    capture = None
    udpClient = None
    visualizer = None

    try:
        ballDetector = BallDetector(settings)
        visualizer = Visualizer()
        capture = create_capture(settings)
        udpClient = UDPClient(port=int(settings.get("udpPort", 31133)))

        context = CameraContext(capture, ballDetector, settings)

        calib_server = CalibServer(context.run_calibration)
        calib_server.start()

        tuner_server = TunerServer(
            context.run_preview,
            context.apply_thresholds,
            context.relax_thresholds,
        )
        tuner_server.start()
        if not tuner_server.wait_ready(5.0):
            debug.log("[tuner] server did not become ready in time")

        output_width = int(settings.get("outputFrameWidth", 160))
        output_height = int(settings.get("outputFrameHeight", 96))
        jpeg_quality = int(settings.get("jpegQuality", 90))

        while True:
            frame, center, circleContour, vertices, distance = context.capture_detect()
            if frame is None:
                debug.log("Error: Failed to capture frame from camera.")
                time.sleep(0.5)
                continue

            frame_height, frame_width = frame.shape[:2]
            if center is not None:
                graph_x, graph_y = Encoder.pixel_to_graph(
                    center[0], center[1], frame_width, frame_height
                )
                debug.log(graph_x, graph_y, distance)
            else:
                debug.log(NO_BALL_COORD, NO_BALL_COORD)

            frame = visualizer.draw(frame, center, circleContour, vertices)

            frame_resized = cv2.resize(
                frame, (output_width, output_height), interpolation=cv2.INTER_AREA
            )

            encoded_json_string = Encoder.encodeData(
                frame_resized,
                center,
                frame_width=frame_width,
                frame_height=frame_height,
                quality=jpeg_quality,
            )

            if encoded_json_string:
                udpClient.send(encoded_json_string)

    except IOError as e:
        debug.log(f"Initialization Error: {e}")
    except KeyboardInterrupt:
        debug.log("Program interrupted by user.")
    except Exception as e:
        debug.log(f"An unexpected error occurred in main loop: {e}")
        if debug.enabled():
            import traceback

            traceback.print_exc()
    finally:
        debug.log("Cleaning up resources...")
        if capture is not None:
            capture.release()
        if udpClient is not None:
            udpClient.close()
        if visualizer is not None:
            visualizer.destroy()
        debug.log("Cleanup complete.")


if __name__ == "__main__":
    main()
