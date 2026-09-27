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
    GOAL_HEIGHT_MM = 10.0

    # Geometry inherited from the 2021 field-hockey route calculation.
    STICK_RIGHT_OFFSET_MM = 100.0
    ORBIT_RADIUS_MM = 200.0

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
            "green": self._empty_target(),
            "route": self._empty_route("ボールまたはゴールを待っています")
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
    def _empty_route(reason):
        return {
            "available": False,
            "provisional": True,
            "reason": reason,
            "command": None
        }

    @staticmethod
    def _normalize_degrees(angle):
        while angle > 180.0:
            angle -= 360.0
        while angle <= -180.0:
            angle += 360.0
        return angle

    def _arc_candidate(self, hit_x, hit_y, left_x, left_y, direction):
        radius = self.ORBIT_RADIUS_MM
        center_x = hit_x + direction * radius * left_x
        center_y = hit_y + direction * radius * left_y
        center_distance_sq = center_x * center_x + center_y * center_y
        if center_distance_sq <= radius * radius:
            return None

        tangent_base_x = (1.0 - radius * radius / center_distance_sq) * center_x
        tangent_base_y = (1.0 - radius * radius / center_distance_sq) * center_y
        factor = radius * math.sqrt(
            center_distance_sq - radius * radius
        ) / center_distance_sq

        for side in (-1.0, 1.0):
            tangent_x = tangent_base_x + side * factor * (-center_y)
            tangent_y = tangent_base_y + side * factor * center_x
            radial_x = tangent_x - center_x
            radial_y = tangent_y - center_y
            if direction > 0:
                travel_x, travel_y = -radial_y, radial_x
            else:
                travel_x, travel_y = radial_y, -radial_x

            # The tangent must be travelled from the robot toward the circle.
            if tangent_x * travel_x + tangent_y * travel_y <= 0:
                continue

            start_angle = math.atan2(radial_y, radial_x)
            end_angle = math.atan2(hit_y - center_y, hit_x - center_x)
            if direction > 0:
                arc_angle = (end_angle - start_angle) % (2.0 * math.pi)
            else:
                arc_angle = (start_angle - end_angle) % (2.0 * math.pi)

            straight = math.hypot(tangent_x, tangent_y)
            return {
                "direction": direction,
                "center_x": center_x,
                "center_y": center_y,
                "tangent_x": tangent_x,
                "tangent_y": tangent_y,
                "first_turn_deg": self._normalize_degrees(
                    math.degrees(math.atan2(tangent_y, tangent_x))
                ),
                "straight_mm": straight,
                "arc_deg": direction * math.degrees(arc_angle),
                "path_length_mm": straight + radius * arc_angle
            }
        return None

    def _plan_route(self, ball, goal):
        if not ball["found"] or not goal["found"]:
            return self._empty_route("ボールまたはゴールを待っています")
        values = (
            ball["robot_x_mm"], ball["robot_y_mm"],
            goal["robot_x_mm"], goal["robot_y_mm"]
        )
        if any(value is None for value in values):
            return self._empty_route("距離・座標を算出できません")

        ball_x, ball_y, goal_x, goal_y = values
        goal_vector_x = goal_x - ball_x
        goal_vector_y = goal_y - ball_y
        goal_distance = math.hypot(goal_vector_x, goal_vector_y)
        if goal_distance < 200.0:
            return self._empty_route("ボールとゴールが近すぎます")

        forward_x = goal_vector_x / goal_distance
        forward_y = goal_vector_y / goal_distance
        left_x, left_y = -forward_y, forward_x

        # With the stick on the right, the robot centre is left of the ball.
        hit_x = ball_x + self.STICK_RIGHT_OFFSET_MM * left_x
        hit_y = ball_y + self.STICK_RIGHT_OFFSET_MM * left_y

        candidates = []
        for direction in (1.0, -1.0):
            candidate = self._arc_candidate(
                hit_x, hit_y, left_x, left_y, direction
            )
            if candidate is not None:
                candidates.append(candidate)
        if not candidates:
            return self._empty_route("回り込み円への接線を作れません")

        route = min(candidates, key=lambda item: item["path_length_mm"])
        command = "zc 0 {:+.2f} 10000 {:.2f} {:.2f} {:+.2f}".format(
            route["first_turn_deg"],
            route["straight_mm"],
            self.ORBIT_RADIUS_MM,
            route["arc_deg"]
        )
        return {
            "available": True,
            "provisional": True,
            "reason": None,
            "command": command,
            "first_turn_deg": round(route["first_turn_deg"], 2),
            "straight_mm": round(route["straight_mm"], 1),
            "orbit_radius_mm": round(self.ORBIT_RADIUS_MM, 1),
            "arc_deg": round(route["arc_deg"], 2),
            "hit_heading_deg": round(
                math.degrees(math.atan2(forward_y, forward_x)), 2
            ),
            "hit_x_mm": round(hit_x, 1),
            "hit_y_mm": round(hit_y, 1),
            "tangent_x_mm": round(route["tangent_x"], 1),
            "tangent_y_mm": round(route["tangent_y"], 1),
            "center_x_mm": round(route["center_x"], 1),
            "center_y_mm": round(route["center_y"], 1),
            "stick_offset_mm": round(self.STICK_RIGHT_OFFSET_MM, 1)
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
        route = self._plan_route(red, green)

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
            "green": green,
            "route": route
        }
        with self.lock:
            self.last_status = status

        return output

    def get_status(self):
        with self.lock:
            status = dict(self.last_status)
            status["red"] = dict(self.last_status["red"])
            status["green"] = dict(self.last_status["green"])
            status["route"] = dict(self.last_status["route"])
            return status
