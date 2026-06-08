from __future__ import annotations

import cv2
import numpy as np

from vision.debug_utils import DebugOverlay
from vision.model_utils import DetectedObject


# Tune these per object
BUN_MIN_AREA_PX    = 800   # larger minimum — bun should be a big blob
BUN_MIN_FILL_RATIO = 0.40  # tighter shape filter
BUN_MIN_CONFIDENCE = 0.60  # drop weak detections

PATTY_MIN_AREA_PX    = 800
PATTY_MIN_FILL_RATIO = 0.40
PATTY_MIN_CONFIDENCE = 0.55


def detect_yellow_block(frame_bgr):
    detections, debug_overlays = [], []
    blurred = cv2.GaussianBlur(frame_bgr, (5, 5), 0)
    hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

    # Tightened — pure yellow only, cuts out cream/beige ceiling
    yellow_hsv_low  = (22, 130, 100)
    yellow_hsv_high = (35, 255, 255)
    mask = cv2.inRange(hsv, yellow_hsv_low, yellow_hsv_high)

    open_kernel  = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
    close_kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (9, 9))
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN,  open_kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, close_kernel)

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    for contour in contours:
        contour_area = float(cv2.contourArea(contour))
        if contour_area < BUN_MIN_AREA_PX:
            continue

        x, y, width, height = cv2.boundingRect(contour)
        fill_ratio = contour_area / float(max(1, width * height))
        if fill_ratio < BUN_MIN_FILL_RATIO:
            continue

        confidence = yellow_detection_score(contour_area, BUN_MIN_AREA_PX, fill_ratio)
        if confidence < BUN_MIN_CONFIDENCE:  # confidence gate
            continue

        detection = DetectedObject(
            class_name="yellow block",
            confidence=confidence,
            x=int(x), y=int(y), width=int(width), height=int(height),
        )
        detection.add_attribute("color", "yellow", 1.0)
        detections.append(detection)
        debug_overlays.append(DebugOverlay(
            color=(0, 255, 255), contour=contour,
            label=f"bun {confidence:.2f}", x=int(x), y=int(y),
        ))

    return detections, debug_overlays


def detect_orange_patty(frame_bgr):
    detections, debug_overlays = [], []
    blurred = cv2.GaussianBlur(frame_bgr, (5, 5), 0)
    hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

    orange_hsv_low  = (5,  140, 100)
    orange_hsv_high = (20, 255, 255)
    mask = cv2.inRange(hsv, orange_hsv_low, orange_hsv_high)

    open_kernel  = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
    close_kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (9, 9))
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN,  open_kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, close_kernel)

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    for contour in contours:
        contour_area = float(cv2.contourArea(contour))
        if contour_area < PATTY_MIN_AREA_PX:
            continue

        x, y, width, height = cv2.boundingRect(contour)
        fill_ratio = contour_area / float(max(1, width * height))
        if fill_ratio < PATTY_MIN_FILL_RATIO:
            continue

        confidence = orange_detection_score(contour_area, PATTY_MIN_AREA_PX, fill_ratio)
        if confidence < PATTY_MIN_CONFIDENCE:  # confidence gate
            continue

        detection = DetectedObject(
            class_name="patty",
            confidence=confidence,
            x=int(x), y=int(y), width=int(width), height=int(height),
        )
        detection.add_attribute("color", "orange", 1.0)
        detections.append(detection)
        debug_overlays.append(DebugOverlay(
            color=(0, 165, 255), contour=contour,
            label=f"patty {confidence:.2f}", x=int(x), y=int(y),
        ))

    return detections, debug_overlays


def yellow_detection_score(contour_area, min_area_px, fill_ratio):
    area_score = min(1.0, contour_area / float(max(1, min_area_px * 4)))
    return max(0.0, min(1.0, 0.55 * area_score + 0.45 * max(0.0, min(1.0, fill_ratio))))


def orange_detection_score(contour_area, min_area_px, fill_ratio):
    area_score = min(1.0, contour_area / float(max(1, min_area_px * 4)))
    return max(0.0, min(1.0, 0.55 * area_score + 0.45 * max(0.0, min(1.0, fill_ratio))))