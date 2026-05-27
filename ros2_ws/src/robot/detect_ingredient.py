from __future__ import annotations

import time

from robot.hardware_map import DEFAULT_FSM_HZ, LED, POSITION_UNIT
from robot.robot import FirmwareState, Robot

# ---------------------------------------------------------------------------
# Configuration
# ---------------------------------------------------------------------------

LED_BRIGHTNESS = 255
LIGHT_HOLD_SEC = 2.0

# ---------------------------------------------------------------------------
# Robot helpers
# ---------------------------------------------------------------------------

def configure_robot(robot: Robot) -> None:
    robot.set_unit(POSITION_UNIT)
    robot.enable_vision()  # subscribe to /vision/detections


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
        robot.set_led(LED.ORANGE, LED_BRIGHTNESS)
        robot.set_led(LED.RED, 0)
    elif ingredient == "patty":
        robot.set_led(LED.RED, LED_BRIGHTNESS)
        robot.set_led(LED.ORANGE, 0)


def scan_for_ingredients(robot: Robot) -> str | None:
    detections = robot.get_detections()
    if not detections:
        return None
    for d in detections:
        if d["class_name"] in ("bun", "yellow block"):
            return "bun"
        if d["class_name"] == "patty":
            return "patty"
    return None

# ---------------------------------------------------------------------------
# Main FSM
# ---------------------------------------------------------------------------

def run(robot: Robot) -> None:
    configure_robot(robot)

    state = "INIT"
    lights_off_at = 0.0
    last_detected = None

    period = 1.0 / float(DEFAULT_FSM_HZ)
    next_tick = time.monotonic()

    while True:
        if state == "INIT":
            start_robot(robot)
            dim_all_leds(robot)
            print("[FSM] WATCHING — place bun or patty in front of camera")
            state = "WATCHING"

        elif state == "WATCHING":
            now = time.monotonic()
            ingredient = scan_for_ingredients(robot)

            if ingredient in ("bun", "patty"):
                show_detected_ingredient_led(robot, ingredient)
                lights_off_at = now + LIGHT_HOLD_SEC
                if ingredient != last_detected:
                    print(f"[VISION] Detected: {ingredient}")
                last_detected = ingredient

            elif lights_off_at > 0.0 and now >= lights_off_at:
                dim_all_leds(robot)
                lights_off_at = 0.0
                if last_detected is not None:
                    print("[VISION] Removed — LEDs off")
                last_detected = None

        next_tick += period
        sleep_s = next_tick - time.monotonic()
        if sleep_s > 0.0:
            time.sleep(sleep_s)
        else:
            next_tick = time.monotonic()