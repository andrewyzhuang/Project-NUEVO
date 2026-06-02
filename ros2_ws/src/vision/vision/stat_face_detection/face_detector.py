import cv2
import numpy as np
from pathlib import Path
from vision.debug_utils import DebugOverlay
from vision.model_utils import DetectedObject

# ---------------------------------------------------------------------------
# Load reference face images once at module level
# ---------------------------------------------------------------------------
_face_detector = cv2.CascadeClassifier(
    cv2.data.haarcascades + "haarcascade_frontalface_default.xml"
)

REFERENCE_SIZE = (100, 100)
MATCH_THRESHOLD = 0.75  # 0-1, higher = stricter; tune this

def _load_reference(path: str):
    img = cv2.imread(path, cv2.IMREAD_GRAYSCALE)
    if img is None:
        print(f"[FACE] Could not load reference image: {path}")
        return None
    # Extract face from reference image if one is detected
    faces = _face_detector.detectMultiScale(img, 1.3, 5)
    if len(faces) == 1:
        x, y, w, h = faces[0]
        img = img[y:y+h, x:x+w]
    return cv2.resize(img, REFERENCE_SIZE)

# Put your two reference images here
_references = {
    "person_1": [ 
        _load_reference("/ros2_ws/src/vision/test/stat_face_detection/ref_person1.png"),
        _load_reference("/ros2_ws/src/vision/test/stat_face_detection/ref_person1a.png"),
    ],
    "person_2": [
        _load_reference("/ros2_ws/src/vision/test/stat_face_detection/ref_person2.png"),
        _load_reference("/ros2_ws/src/vision/test/stat_face_detection/ref_person2a.png"),
    ]
                 

}


def _match_score(face_crop: np.ndarray) -> tuple[str, float]:
    """Compare face crop against all references, return best match and score."""
    face_resized = cv2.resize(face_crop, REFERENCE_SIZE)
    
    best_name  = "unknown"
    best_score = 0.0

    for name, ref in _references.items():
        if ref is None:
            continue
        # Normalized cross-correlation: 1.0 = perfect match, -1.0 = inverse
        result = cv2.matchTemplate(
            face_resized.astype(np.float32),
            ref.astype(np.float32),
            cv2.TM_CCOEFF_NORMED,
        )
        score = float(result[0][0])
        if score > best_score:
            best_score = score
            best_name  = name

    if best_score < MATCH_THRESHOLD:
        return "unknown", best_score
    return best_name, best_score


def detect_person(frame_bgr) -> tuple[list[DetectedObject], list[DebugOverlay]]:
    """Detect faces and match against reference images."""
    detections: list[DetectedObject] = []
    debug_overlays: list[DebugOverlay] = []

    gray  = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2GRAY)
    faces = _face_detector.detectMultiScale(gray, scaleFactor=1.3, minNeighbors=5)

    for (x, y, w, h) in faces:
        face_crop      = gray[y:y+h, x:x+w]
        name, score    = _match_score(face_crop)

        detection = DetectedObject(
            class_name=name,
            confidence=float(score),
            x=int(x), y=int(y), width=int(w), height=int(h),
        )
        detection.add_attribute("identity", name, float(score))
        detections.append(detection)
        debug_overlays.append(DebugOverlay(
            color=(255, 0, 255),
            contour=None,
            label=f"{name} {score:.2f}",
            x=int(x), y=int(y),
        ))

    return detections, debug_overlays