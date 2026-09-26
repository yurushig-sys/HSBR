"""Detection-only support for the HSBR field hockey application.

Python 3.7 compatible.  This module never sends motion commands.
"""

import threading
import time

import cv2
import numpy as np


class FieldHockeyDetector(object):
    HORIZONTAL_FOV_DEG = 48.0

    def __init__(self, min_area_ratio=0.0001):
        self.min_area_ratio = min_area_ratio
        self.lock = threading.Lock()
        self.last_status = {
            "ok": True,
            "mode": "detection",
            "processed_at": 0.0,
            "frame_width": 0,
            "frame_height": 0,
            "red": self._empty_target(),
            "green": self._empty_target()
        }

    @staticmethod
    def _empty_target():
        return {
            "found": False,
            "x": None,
            "y": None,
            "area": 0.0,
            "bearing_deg": None
        }

    @staticmethod
    def _clean_mask(mask):
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        return cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

    def _find_largest(self, mask, width, height):
        result = cv2.findContours(
            mask,
            cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_SIMPLE
        )
        contours = result[-2]

        if not contours:
            return self._empty_target(), None

        contour = max(contours, key=cv2.contourArea)
        area = float(cv2.contourArea(contour))
        if area < width * height * self.min_area_ratio:
            return self._empty_target(), None

        moments = cv2.moments(contour)
        if moments["m00"] == 0:
            return self._empty_target(), None

        x = int(moments["m10"] / moments["m00"])
        y = int(moments["m01"] / moments["m00"])
        bearing = ((width / 2.0 - x) / (width / 2.0)) * (
            self.HORIZONTAL_FOV_DEG / 2.0
        )

        target = {
            "found": True,
            "x": x,
            "y": y,
            "area": round(area, 1),
            # Positive is left, matching the 2021 field-hockey program.
            "bearing_deg": round(float(bearing), 1)
        }
        return target, contour

    @staticmethod
    def _draw_target(frame, target, contour, color, label, circle=False):
        if not target["found"] or contour is None:
            return

        if circle:
            (cx, cy), radius = cv2.minEnclosingCircle(contour)
            cv2.circle(
                frame,
                (int(cx), int(cy)),
                max(2, int(radius)),
                color,
                3
            )
            label_y = max(22, int(cy - radius - 8))
            label_x = max(4, int(cx - radius))
        else:
            x, y, w, h = cv2.boundingRect(contour)
            cv2.rectangle(frame, (x, y), (x + w, y + h), color, 3)
            label_x = x
            label_y = max(22, y - 8)

        cv2.drawMarker(
            frame,
            (target["x"], target["y"]),
            color,
            cv2.MARKER_CROSS,
            18,
            2
        )
        text = "{} x={} y={} a={:.0f} th={:+.1f}".format(
            label,
            target["x"],
            target["y"],
            target["area"],
            target["bearing_deg"]
        )
        cv2.putText(
            frame,
            text,
            (label_x, label_y),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            color,
            2,
            cv2.LINE_AA
        )

    def process(self, frame):
        output = frame.copy()
        height, width = output.shape[:2]
        hsv = cv2.cvtColor(output, cv2.COLOR_BGR2HSV)

        # Red wraps around the HSV hue boundary, so combine two ranges.
        red_low = cv2.inRange(
            hsv,
            np.array([0, 100, 80]),
            np.array([10, 255, 255])
        )
        red_high = cv2.inRange(
            hsv,
            np.array([165, 100, 80]),
            np.array([180, 255, 255])
        )
        red_mask = self._clean_mask(cv2.bitwise_or(red_low, red_high))

        # Initial range inherited from the 2021 field-hockey application.
        green_mask = cv2.inRange(
            hsv,
            np.array([60, 50, 50]),
            np.array([90, 255, 255])
        )
        green_mask = self._clean_mask(green_mask)

        red, red_contour = self._find_largest(red_mask, width, height)
        green, green_contour = self._find_largest(
            green_mask,
            width,
            height
        )

        cv2.line(output, (width // 2, 0), (width // 2, height), (210, 210, 210), 1)
        cv2.line(output, (0, height // 2), (width, height // 2), (210, 210, 210), 1)
        self._draw_target(output, red, red_contour, (0, 0, 255), "BALL", True)
        self._draw_target(output, green, green_contour, (0, 255, 0), "GOAL")

        status = {
            "ok": True,
            "mode": "detection",
            "processed_at": round(time.time(), 3),
            "frame_width": width,
            "frame_height": height,
            "red": red,
            "green": green
        }
        with self.lock:
            self.last_status = status

        return output

    def get_status(self):
        with self.lock:
            status = dict(self.last_status)
            status["red"] = dict(self.last_status["red"])
            status["green"] = dict(self.last_status["green"])
            return status
