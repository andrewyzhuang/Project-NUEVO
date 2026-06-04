from __future__ import annotations

import time

from robot.hardware_map import DEFAULT_FSM_HZ, LED, POSITION_UNIT
from robot.robot import FirmwareState, Robot

LED_BRIGHTNESS = 255
LIGHT_HOLD_SEC = 2.0


def configure_robot(robot: Robot) -> None:
    robot.set_unit(POSITION_UNIT)
    robot.enable_vision()


def start_robot(robot: Robot) -> None:
    current = robot.get_state()
    if current in (FirmwareState.ESTOP, FirmwareState.ERROR):
        robot.reset_estop()
    robot.set_state(FirmwareState.RUNNING)


def dim_all_leds(robot: Robot) -> None:
    for led in (LED.RED, LED.GREEN, LED.BLUE, LED.ORANGE, LED.PURPLE):
        robot.set_led(led, 0)


def show_person_led(robot: Robot, person: str) -> None:
    if person == "person_1":
        robot.set_led(LED.GREEN, LED_BRIGHTNESS)
        robot.set_led(LED.BLUE, 0)
    elif person == "person_2":
        robot.set_led(LED.BLUE, LED_BRIGHTNESS)
        robot.set_led(LED.GREEN, 0)


def scan_for_person(robot: Robot) -> str | None:
    detections = robot.get_detections()
    if not detections:
        return None
    for d in detections:
        if d["class_name"] in ("person_1", "person_2"):
            return d["class_name"]
    return None


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
            print("[FSM] WATCHING — show person 1 or 2 to camera")
            state = "WATCHING"

        elif state == "WATCHING":
            now = time.monotonic()
            person = scan_for_person(robot)

            if person in ("person_1", "person_2"):
                show_person_led(robot, person)
                lights_off_at = now + LIGHT_HOLD_SEC
                if person != last_detected:
                    print(f"[VISION] Detected: {person}")
                last_detected = person

            elif lights_off_at > 0.0 and now >= lights_off_at:
                dim_all_leds(robot)
                lights_off_at = 0.0
                if last_detected is not None:
                    print("[VISION] Person left — LEDs off")
                last_detected = None

        next_tick += period
        sleep_s = next_tick - time.monotonic()
        if sleep_s > 0.0:
            time.sleep(sleep_s)
        else:
            next_tick = time.monotonic()

