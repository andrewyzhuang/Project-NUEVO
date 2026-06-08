"""
manipulation.py — stepper lift + servo gripper, no limit switch
===============================================================
Uses:
  - Stepper 1 as a vertical lift
  - Servo 1 as a gripper

No limit switch required. Place the platform at its lowest position
by hand before powering on. All moves are relative, so the stepper
counts steps from wherever it starts.

HOW TO RUN
----------
Copy this file over main.py, then restart the robot node:

    cp examples/manipulation.py main.py
    ros2 run robot robot

WHAT THE ROBOT DOES
-------------------
Press BTN_1 to run one pick sequence:

  1. Raise the platform
  2. Open the gripper
  3. Lower the platform
  4. Close the gripper
  5. Raise the platform

If the sequence is interrupted or times out, the robot returns to
INIT and prompts you to manually lower the platform before retrying.
This prevents the stepper from losing position and over-extending.

WHAT THIS TEACHES
-----------------
1. Running a stepper with only relative moves (no homing required)
2. `set_servo()` for gripper angle control
3. Writing a simple blocking actuator sequence with `time.sleep()`
4. Safe failure handling — disabling actuators and resetting state
   on any timeout
"""

from __future__ import annotations

import time

from robot.hardware_map import (
    Button,
    DEFAULT_FSM_HZ,
    LED,
    POSITION_UNIT,
    ServoChannel,
    StepMoveType,
    Stepper,
)
from robot.robot import FirmwareState, Robot


# ---------------------------------------------------------------------------
# Actuator configuration — edit these to match your build
# ---------------------------------------------------------------------------

# Servo 1 — gripper jaw
GRIPPER_SERVO = ServoChannel.CH_1
GRIPPER_OPEN_DEG = 15.0
GRIPPER_CLOSE_DEG = 120.0
GRIPPER_SETTLE_S = 1.0          # seconds to wait after a servo move

# Stepper 1 — vertical lift
LIFT_STEPPER = Stepper.STEPPER_1
LIFT_UP_STEPS = 2000            # steps from bottom to top
LIFT_MAX_VELOCITY = 800         # steps/s
LIFT_ACCELERATION = 400         # steps/s²
LIFT_MOVE_TIMEOUT_S = 10.0      # max seconds allowed for one lift move


# ---------------------------------------------------------------------------
# Robot setup helpers
# ---------------------------------------------------------------------------

def configure_robot(robot: Robot) -> None:
    robot.set_unit(POSITION_UNIT)


def start_robot(robot: Robot) -> None:
    current = robot.get_state()
    if current in (FirmwareState.ESTOP, FirmwareState.ERROR):
        robot.reset_estop()
    robot.set_state(FirmwareState.RUNNING)


def show_idle_leds(robot: Robot) -> None:
    robot.set_led(LED.ORANGE, 200)
    robot.set_led(LED.GREEN, 0)


def show_running_leds(robot: Robot) -> None:
    robot.set_led(LED.ORANGE, 0)
    robot.set_led(LED.GREEN, 200)


# ---------------------------------------------------------------------------
# Pick sequence
# ---------------------------------------------------------------------------

def run_pick_sequence(robot: Robot) -> bool:
    """
    Run one blocking pick sequence.

    Returns True on success, False if any step times out.
    On failure all actuators are disabled before returning.
    """
    robot.step_set_config(
        LIFT_STEPPER,
        max_velocity=LIFT_MAX_VELOCITY,
        acceleration=LIFT_ACCELERATION,
    )
    robot.enable_servo(GRIPPER_SERVO)
    robot.step_enable(LIFT_STEPPER)

    # --- 1. Raise platform ---------------------------------------------------
    print("[SEQ] raise platform")
    if not robot.step_move(
        LIFT_STEPPER,
        steps=LIFT_UP_STEPS,
        move_type=StepMoveType.RELATIVE,
        blocking=True,
        timeout=LIFT_MOVE_TIMEOUT_S,
    ):
        print("[warn] lift failed to raise — check stepper wiring")
        robot.step_disable(LIFT_STEPPER)
        robot.disable_servo(GRIPPER_SERVO)
        return False

    # --- 2. Open gripper -----------------------------------------------------
    print("[SEQ] open gripper")
    robot.set_servo(GRIPPER_SERVO, GRIPPER_OPEN_DEG)
    time.sleep(GRIPPER_SETTLE_S)

    # --- 3. Lower platform ---------------------------------------------------
    print("[SEQ] lower platform")
    if not robot.step_move(
        LIFT_STEPPER,
        steps=-LIFT_UP_STEPS,
        move_type=StepMoveType.RELATIVE,
        blocking=True,
        timeout=LIFT_MOVE_TIMEOUT_S,
    ):
        print("[warn] lift failed to lower")
        robot.step_disable(LIFT_STEPPER)
        robot.disable_servo(GRIPPER_SERVO)
        return False

    # --- 4. Close gripper ----------------------------------------------------
    print("[SEQ] close gripper")
    robot.set_servo(GRIPPER_SERVO, GRIPPER_CLOSE_DEG)
    time.sleep(GRIPPER_SETTLE_S)

    # --- 5. Raise platform with part -----------------------------------------
    print("[SEQ] raise platform with part")
    if not robot.step_move(
        LIFT_STEPPER,
        steps=LIFT_UP_STEPS,
        move_type=StepMoveType.RELATIVE,
        blocking=True,
        timeout=LIFT_MOVE_TIMEOUT_S,
    ):
        print("[warn] lift failed to raise with part")
        robot.step_disable(LIFT_STEPPER)
        robot.disable_servo(GRIPPER_SERVO)
        return False

    # --- Clean up ------------------------------------------------------------
    robot.step_disable(LIFT_STEPPER)
    robot.disable_servo(GRIPPER_SERVO)
    return True


# ---------------------------------------------------------------------------
# Main FSM loop
# ---------------------------------------------------------------------------

def run(robot: Robot) -> None:
    configure_robot(robot)

    state = "INIT"
    period = 1.0 / float(DEFAULT_FSM_HZ)
    next_tick = time.monotonic()

    while True:

        if state == "INIT":
            start_robot(robot)
            show_idle_leds(robot)
            print("[FSM] IDLE — manually lower the platform to its bottom")
            print("            position, then press BTN_1 to run the sequence")
            print(f"[CFG] gripper open={GRIPPER_OPEN_DEG:.0f}°  "
                  f"close={GRIPPER_CLOSE_DEG:.0f}°")
            print(f"[CFG] lift steps={LIFT_UP_STEPS}  "
                  f"max_vel={LIFT_MAX_VELOCITY} steps/s")
            state = "IDLE"

        elif state == "IDLE":
            if robot.was_button_pressed(Button.BTN_1):
                show_running_leds(robot)
                print("[FSM] RUN_SEQUENCE")
                state = "RUN_SEQUENCE"

        elif state == "RUN_SEQUENCE":
            ok = run_pick_sequence(robot)
            show_idle_leds(robot)
            if ok:
                print("[FSM] IDLE — sequence complete")
                state = "IDLE"
            else:
                # Go back to INIT so the operator must confirm the platform
                # is back at the bottom before trying again.
                print("[FSM] INIT — sequence failed; manually lower the platform")
                state = "INIT"

        next_tick += period
        sleep_s = next_tick - time.monotonic()
        if sleep_s > 0.0:
            time.sleep(sleep_s)
        else:
            next_tick = time.monotonic()