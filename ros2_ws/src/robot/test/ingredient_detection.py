###Detect ingredient (using range of colors, masking)

from __future__ import annotations

import time
import cv2
import numpy as np

from robot.hardware_map import DEFAULT_FSM_HZ, LED, POSITION_UNIT
from robot.robot import FirmwareState, Robot

# ---------------------------------------------------------------------------
# Configuration
# ---------------------------------------------------------------------------

LED_BRIGHTNESS = 255
LIGHT_HOLD_SEC = 2.0

# Define your calibrated HSV ranges (Modify these based on your trackbar tests!)
# Format: [Hue, Saturation, Value]
LOWER_BUN = np.array([10, 50, 40])
UPPER_BUN = np.array([25, 255, 255])

LOWER_PATTY = np.array([0, 50, 20])
UPPER_PATTY = np.array([15, 180, 120])

# Global camera object
camera = None

# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def configure_robot(robot: Robot) -> None:
    global camera
    robot.set_unit(POSITION_UNIT)
    # Note: We do NOT call robot.enable_vision() because we are bypasssing the ML node
    
    # Initialize the physical Pi Camera or USB Camera directly
    camera = cv2.VideoCapture(0)
    if not camera.isOpened():
        print("[ERROR] Could not open camera source.")


def start_robot(robot: Robot) -> None:
    current = robot.get_state()
    if current in (FirmwareState.ESTOP, FirmwareState.ERROR):
        robot.reset_estop()
    robot.set_state(FirmwareState.RUNNING)


def dim_all_leds(robot: Robot) -> None:
    for led in (LED.RED, LED.GREEN, LED.BLUE, LED.ORANGE, LED.PURPLE):
        robot.set_led(led, 0)


def show_detected_ingredient_led(robot: Robot, ingredient: str) -> None:
    if ingredient == "bun":
        robot.set_led(LED.ORANGE, LED_BRIGHTNESS)  # Orange for bun
        robot.set_led(LED.RED, 0)
    elif ingredient == "patty":
        robot.set_led(LED.ORANGE, 0)
        robot.set_led(LED.RED, LED_BRIGHTNESS)     # Red/Dark for patty


def process_image_for_ingredient(frame, lower_hsv, upper_hsv) -> bool:
    """Returns True if a round object within the HSV range is detected."""
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, lower_hsv, upper_hsv)
    
    # Clean up image noise
    kernel = np.ones((5,5), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    for contour in contours:
        area = cv2.contourArea(contour)
        if area > 600: # Filter out tiny background dots
            perimeter = cv2.arcLength(contour, True)
            if perimeter == 0: continue
            
            # Circularity calculation
            circularity = 4 * np.pi * (area / (perimeter * perimeter))
            
            # If it's reasonably round, we found it!
            if 0.65 < circularity < 1.35:
                return True
                
    return False


def scan_for_ingredients() -> str | None:
    """Manually captures a camera frame and looks for buns or patties."""
    global camera
    if camera is None or not camera.isOpened():
        return None

    ret, frame = camera.read()
    if not ret:
        return None

    # Check for bun first
    if process_image_for_ingredient(frame, LOWER_BUN, UPPER_BUN):
        return "bun"
        
    # Check for patty
    if process_image_for_ingredient(frame, LOWER_PATTY, UPPER_PATTY):
        return "patty"

    return None


# ---------------------------------------------------------------------------
# run() - entry point called by the robot node
# ---------------------------------------------------------------------------

def run(robot: Robot) -> None:
    global camera
    configure_robot(robot)

    state = "INIT"
    lights_off_at = 0.0
    last_detected_ingredient = None

    period = 1.0 / float(DEFAULT_FSM_HZ)
    next_tick = time.monotonic()

    try:
        while True:
            # -- INIT -----------------------------------------------------------
            if state == "INIT":
                start_robot(robot)
                dim_all_leds(robot)
                print("[FSM] WATCHING - Place a plastic bun or patty in front of the camera")
                state = "WATCHING"

            # -- WATCHING -------------------------------------------------------
            elif state == "WATCHING":
                now = time.monotonic()

                # Call our custom CV function instead of the old pre-trained model method
                ingredient = scan_for_ingredients()

                if ingredient in ("bun", "patty"):
                    show_detected_ingredient_led(robot, ingredient)
                    lights_off_at = now + LIGHT_HOLD_SEC

                    if ingredient != last_detected_ingredient:
                        print(f"[VISION] Handled via CV - Detected: {ingredient}")
                    last_detected_ingredient = ingredient

                elif lights_off_at > 0.0 and now >= lights_off_at:
                    dim_all_leds(robot)
                    lights_off_at = 0.0
                    if last_detected_ingredient is not None:
                        print("[VISION] Ingredient removed - LEDs off")
                    last_detected_ingredient = None

            # -- Tick-rate control ---------------------------------------------
            next_tick += period
            sleep_s = next_tick - time.monotonic()
            if sleep_s > 0.0:
                time.sleep(sleep_s)
            else:
                next_tick = time.monotonic()
                
    finally:
        if camera is not None:
            camera.release()