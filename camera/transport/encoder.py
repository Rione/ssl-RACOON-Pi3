"""Encodes detection results into the JSON payload consumed by the Go side.

The schema (frame/x/y/isball) matches internal/state/state.go ImageData.
Coordinates use a graph-style system: origin at the frame center, x right,
y up (positive y is above the center).
"""

import base64
import json

import cv2

from camera import debug

NO_BALL_COORD = 9999


class Encoder:
    @staticmethod
    def pixel_to_graph(pixel_x, pixel_y, frame_width, frame_height):
        """Convert image pixel coords to graph coords (origin at center, y up)."""
        return (
            float(pixel_x) - frame_width / 2.0,
            frame_height / 2.0 - float(pixel_y),
        )

    @staticmethod
    def encodeData(
        frame,
        center=None,
        frame_width=None,
        frame_height=None,
        quality=90,
    ):
        if frame is None or frame.size == 0:
            debug.log("Error: Cannot encode empty frame.")
            return None

        encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), quality]
        result, encoded_image = cv2.imencode(".jpg", frame, encode_param)

        if not result:
            debug.log("Error: Failed to encode image to JPEG.")
            return None

        frame_bytes_b64 = base64.b64encode(encoded_image.tobytes()).decode("utf-8")

        if frame_width is None or frame_height is None:
            frame_height, frame_width = frame.shape[:2]

        if center is not None:
            x_coord, y_coord = Encoder.pixel_to_graph(
                center[0], center[1], frame_width, frame_height
            )
            isball = True
        else:
            x_coord = NO_BALL_COORD
            y_coord = NO_BALL_COORD
            isball = False

        data = {
            "frame": frame_bytes_b64,
            "x": x_coord,
            "y": y_coord,
            "isball": isball,
            "frameWidth": int(frame_width),
            "frameHeight": int(frame_height),
        }

        try:
            return json.dumps(data)
        except TypeError as e:
            debug.log(f"Error serializing data to JSON: {e}")
            return None
