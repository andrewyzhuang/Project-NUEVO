from __future__ import annotations

import cv2
import numpy as np
from pathlib import Path
from vision.debug_utils import DebugOverlay
from vision.model_utils import DetectedObject

REFERENCE_SIZE = (100, 100)
MATCH_THRESHOLD = 0.50

_HERE = Path(__file__).parent

def _load_reference(path: str):
    img = cv2.imread(path, cv2.IMREAD_GRAYSCALE)
    if img is None:
        print(f"[FACE] Could not load reference image: {path}")
        return None
    return cv2.resize(img, REFERENCE_SIZE)

_references = {
    "person_1": [
        _load_reference(str(_HERE / "ref_person1.png")),
        #_load_reference(str(_HERE / "ref_person1a.png")),
    ],
    "person_2": [
        _load_reference(str(_HERE / "ref_person2.png")),
        #_load_reference(str(_HERE / "ref_person2a.png")),
    ],
}
_references = {
    name: [r for r in refs if r is not None]
    for name, refs in _references.items()
}


def _match_score(crop: np.ndarray) -> tuple[str, float]:
    """Compare a person crop against all references, return best match and score."""
    crop_resized = cv2.resize(crop, REFERENCE_SIZE)
    best_name  = "unknown"
    best_score = 0.0

    for name, refs in _references.items():
        if not refs:
            continue
        scores = []
        for ref in refs:
            result = cv2.matchTemplate(
                crop_resized.astype(np.float32),
                ref.astype(np.float32),
                cv2.TM_CCOEFF_NORMED,
            )
            scores.append(float(result[0][0]))
        avg = sum(scores) / len(scores)
        if avg > best_score:
            best_score = avg
            best_name  = name

    if best_score < MATCH_THRESHOLD:
        return "unknown", best_score
    return best_name, best_score


def classify_person_crop(
    person_crop_bgr: np.ndarray,
    x: int, y: int, w: int, h: int,
) -> tuple[DetectedObject | None, DebugOverlay | None]:
    """
    Takes a YOLO person crop and identifies if it's person_1 or person_2.
    Called from vision_node.py inside the YOLO person branch.
    """
    if person_crop_bgr.size == 0:
        return None, None

    gray = cv2.cvtColor(person_crop_bgr, cv2.COLOR_BGR2GRAY)
    name, score = _match_score(gray)

    if name == "unknown":
        return None, None

    detection = DetectedObject(
        class_name=name,
        confidence=float(score),
        x=x, y=y, width=w, height=h,
    )
    detection.add_attribute("identity", name, float(score))

    overlay = DebugOverlay(
        color=(255, 0, 255),
        contour=None,
        label=f"{name} {score:.2f}",
        x=x, y=y,
    )

    return detection, overlay