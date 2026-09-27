"""Detection-only support for the HSBR field hockey application.

Python 3.7 compatible.  This module never sends motion commands.
"""

import math
import threading
import time

import cv2
import numpy as np


class FieldHockeyDetector(object):
    # Provisional calibration from five measurements at X=500 mm.
    # cx is the optical centre and fx is the horizontal focal length in pixels.
    CAMERA_CX_PX = 320.0
    CAMERA_FX_PX = 525.0

    # Provisional vertical-distance calibration.  These values are used for
    # display only and must not yet be used to issue motion commands.
    CAMERA_CY_PX = 240.0
    CAMERA_FY_PX = 779.0
    CAMERA_HEIGHT_MM = 170.0
    CAMERA_DOWN_PITCH_DEG = 6.83
    BALL_CENTER_HEIGHT_MM = 60.0
    GOAL_HEIGHT_MM = 0.0

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
            "bearing_deg": None,
            # Filled after vertical-distance calibration.
            "distance_mm": None,
            "robot_x_mm": None,
            "robot_y_mm": None
        }

    @staticmethod
    def _clean_mask(mask):
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        return cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

    def _find_largest(self, mask, width, height, target_height_mm):
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
        # Positive is left.  atan is used instead of a linear FOV conversion.
        scale = width / 640.0
        camera_cx = self.CAMERA_CX_PX * scale
        camera_fx = self.CAMERA_FX_PX * scale
        bearing = math.degrees(math.atan((camera_cx - x) / camera_fx))

        vertical_scale = height / 480.0
        camera_cy = self.CAMERA_CY_PX * vertical_scale
        camera_fy = self.CAMERA_FY_PX * vertical_scale
        down_angle = math.radians(self.CAMERA_DOWN_PITCH_DEG) + math.atan(
            (y - camera_cy) / camera_fy
        )

        distance_mm = None
        robot_x_mm = None
        robot_y_mm = None
        height_difference = self.CAMERA_HEIGHT_MM - target_height_mm
        if down_angle > math.radians(0.5) and height_difference > 0:
            robot_x_mm = height_difference / math.tan(down_angle)
            robot_y_mm = robot_x_mm * math.tan(math.radians(bearing))
            distance_mm = math.hypot(robot_x_mm, robot_y_mm)

        target = {
            "found": True,
            "x": x,
            "y": y,
            "area": round(area, 1),
            "bearing_deg": round(float(bearing), 1),
            "distance_mm": (
                round(float(distance_mm), 1)
                if distance_mm is not None else None
            ),
            "robot_x_mm": (
                round(float(robot_x_mm), 1)
                if robot_x_mm is not None else None
            ),
            "robot_y_mm": (
                round(float(robot_y_mm), 1)
                if robot_y_mm is not None else None
            )
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
        text = "{} th={:+.1f}deg".format(
            label,
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

        red, red_contour = self._find_largest(
            red_mask,
            width,
            height,
            self.BALL_CENTER_HEIGHT_MM
        )
        green, green_contour = self._find_largest(
            green_mask,
            width,
            height,
            self.GOAL_HEIGHT_MM
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
