"""
pick_sequence.py — integrated motion + manipulation pick sequence
=================================================================
Hardware:
  - Stepper 1  : vertical lift (no limit switch, start platform at bottom)
  - Servo 1    : gripper jaw

Layout:
  Three pick points (P1, P2, P3) are arranged in a line.
  The robot starts facing toward the line (forward = toward the points).
  All picks use a hardcoded LEFT turn to face the item.
  Assuming that the stepper starts at the top of the plate

Pick order: P1 → P2 → P3

Sequence:
  Start → forward to P1 → turn left → pick → turn right (revert) →
  forward to P2 → turn left → pick → turn right (revert) →
  forward to P3 → turn left → pick → turn right (revert) → done

HOW TO RUN
----------
    cp assembly.py main.py
    ros2 run robot robot

Press BTN_1 to start. On any actuator failure the robot stops and returns
to INIT. BTN_2 cancels the sequence mid-run and returns to IDLE.
"""

from __future__ import annotations

import time

from robot.hardware_map import (
    Button,
    DEFAULT_FSM_HZ,
    LED,
    INITIAL_THETA_DEG,
    LEFT_WHEEL_DIR_INVERTED,
    LEFT_WHEEL_MOTOR,
    POSITION_UNIT,
    RIGHT_WHEEL_DIR_INVERTED,
    RIGHT_WHEEL_MOTOR,
    ServoChannel,
    StepMoveType,
    Stepper,
    WHEEL_BASE,
    WHEEL_DIAMETER,
)
from robot.robot import FirmwareState, Robot


# ---------------------------------------------------------------------------
# Tunable constants — fill these in for your build
# ---------------------------------------------------------------------------

# --- Gripper (Servo 1) ---
GRIPPER_SERVO       = ServoChannel.CH_1
GRIPPER_OPEN_DEG    = 60          # degrees — jaw open
GRIPPER_CLOSE_DEG   = 170.0         # degrees — jaw closed
GRIPPER_SETTLE_S    = 1.0           # seconds to wait after each servo move

# --- Lift (Stepper 1) ---
LIFT_STEPPER        = Stepper.STEPPER_1
LIFT_UP_STEPS       = 2100          # steps from bottom to top
PICK_LIFT_STEPS     = 1200          # moves from bottom between picks
INIT_LOWER_STEPS    = int(PICK_LIFT_STEPS - LIFT_UP_STEPS)
LIFT_MAX_VELOCITY   = 800           # steps/s
LIFT_ACCELERATION   = 400           # steps/s²
LIFT_MOVE_TIMEOUT_S = 10.0          # max seconds per lift move

# --- Drive ---
DRIVE_VELOCITY_MM_S  = 100.0        # mm/s for all forward/backward moves
DRIVE_TOLERANCE_MM   = 20.0         # mm position tolerance
TURN_TOLERANCE_DEG   = 3.0          # degrees heading tolerance

# --- Point geometry (set these to match your actual field) ---
# Distance from start to P1 (first point visited)
DIST_START_TO_P1_MM  = 950       # mm — placeholder
# Distance from P1 forward to P2
DIST_P1_TO_P2_MM     = 150.0        # mm — placeholder
# Distance from P2 forward to P3
DIST_P2_TO_P3_MM     = 150.0        # mm — placeholder
# How far to move toward the pick item at each point
PICK_SIDE_OFFSET_MM  = 5.0        # mm — placeholder


# ---------------------------------------------------------------------------
# LED helpers
# ---------------------------------------------------------------------------

def show_idle_leds(robot: Robot) -> None:
    robot.set_led(LED.ORANGE, 200)
    robot.set_led(LED.GREEN, 0)


def show_running_leds(robot: Robot) -> None:
    robot.set_led(LED.ORANGE, 0)
    robot.set_led(LED.GREEN, 200)


# ---------------------------------------------------------------------------
# Robot startup
# ---------------------------------------------------------------------------

def configure_robot(robot: Robot) -> None:
    robot.set_unit(POSITION_UNIT)
    robot.set_odometry_parameters(
        wheel_diameter=WHEEL_DIAMETER,
        wheel_base=WHEEL_BASE,
        initial_theta_deg=INITIAL_THETA_DEG,
        left_motor_id=LEFT_WHEEL_MOTOR,
        left_motor_dir_inverted=LEFT_WHEEL_DIR_INVERTED,
        right_motor_id=RIGHT_WHEEL_MOTOR,
        right_motor_dir_inverted=RIGHT_WHEEL_DIR_INVERTED,
    )


def start_robot(robot: Robot) -> None:
    current = robot.get_state()
    if current in (FirmwareState.ESTOP, FirmwareState.ERROR):
        robot.reset_estop()
    robot.set_state(FirmwareState.RUNNING)


# ---------------------------------------------------------------------------
# Cancel flag
# ---------------------------------------------------------------------------

class CancelFlag:
    """Simple wrapper so functions can signal and check for cancellation."""
    def __init__(self) -> None:
        self.cancelled = False

    def cancel(self) -> None:
        self.cancelled = True

    def is_set(self) -> bool:
        return self.cancelled


# ---------------------------------------------------------------------------
# Manipulation primitives
# ---------------------------------------------------------------------------

def lift_move(robot: Robot, steps: int) -> bool:
    """Move the lift by `steps` (positive = up, negative = down)."""
    return robot.step_move(
        LIFT_STEPPER,
        steps=steps,
        move_type=StepMoveType.RELATIVE,
        blocking=True,
        timeout=LIFT_MOVE_TIMEOUT_S,
    )


def perform_pick(robot: Robot) -> bool:
    """
    Open gripper → lower platform → close gripper → raise platform.
    Platform must already be raised before calling this.
    Returns False and disables actuators on any failure.
    """
    robot.enable_servo(GRIPPER_SERVO)
    robot.step_set_config(
        LIFT_STEPPER,
        max_velocity=LIFT_MAX_VELOCITY,
        acceleration=LIFT_ACCELERATION,
    )
    robot.step_enable(LIFT_STEPPER)

    print("[MANIP] open gripper")
    robot.set_servo(GRIPPER_SERVO, GRIPPER_OPEN_DEG)
    time.sleep(GRIPPER_SETTLE_S)

    print("[MANIP] lower platform")
    if not lift_move(robot, -PICK_LIFT_STEPS):
        print("[warn] lift failed to lower")
        robot.step_disable(LIFT_STEPPER)
        robot.disable_servo(GRIPPER_SERVO)
        return False

    print("[MANIP] close gripper")
    robot.set_servo(GRIPPER_SERVO, GRIPPER_CLOSE_DEG)
    time.sleep(GRIPPER_SETTLE_S)

    print("[MANIP] raise platform")
    if not lift_move(robot, PICK_LIFT_STEPS):
        print("[warn] lift failed to raise")
        robot.step_disable(LIFT_STEPPER)
        robot.disable_servo(GRIPPER_SERVO)
        return False

    robot.step_disable(LIFT_STEPPER)
    robot.disable_servo(GRIPPER_SERVO)
    return True


# ---------------------------------------------------------------------------
# Per-point pick routine — always turns left
# ---------------------------------------------------------------------------

def pick_at_point(robot: Robot, label: str, cancel: CancelFlag) -> bool:
    """
    At the current position:
      1. Turn left 90°
      2. Move forward PICK_SIDE_OFFSET_MM toward item
      3. Perform pick (open → lower → close → raise)
      4. Move backward PICK_SIDE_OFFSET_MM back to line
      5. Turn right 90° (revert left turn)

    Returns False on any failure or cancellation.
    """
    if cancel.is_set():
        return False

    print(f"[MOTION] {label} — turn left 90°")
    robot.turn_by(
        delta_deg=90.0,
        blocking=True,
        tolerance_deg=TURN_TOLERANCE_DEG,
    )

    if cancel.is_set():
        return False

    print(f"[MOTION] {label} — move {PICK_SIDE_OFFSET_MM:.0f} mm toward item")
    robot.move_forward(
        distance=PICK_SIDE_OFFSET_MM,
        velocity=DRIVE_VELOCITY_MM_S,
        tolerance=DRIVE_TOLERANCE_MM,
        blocking=True,
    )

    if cancel.is_set():
        return False

    if not perform_pick(robot):
        return False

    if cancel.is_set():
        return False

    print(f"[MOTION] {label} — return {PICK_SIDE_OFFSET_MM:.0f} mm to line")
    robot.move_backward(
        distance=PICK_SIDE_OFFSET_MM,
        velocity=DRIVE_VELOCITY_MM_S,
        tolerance=DRIVE_TOLERANCE_MM,
        blocking=True,
    )

    if cancel.is_set():
        return False

    print(f"[MOTION] {label} — turn right 90° (revert)")
    robot.turn_by(
        delta_deg=-90.0,
        blocking=True,
        tolerance_deg=TURN_TOLERANCE_DEG,
    )

    print(f"[MOTION] {label} pick complete")
    return True


# ---------------------------------------------------------------------------
# Full pick sequence:  P1 → P2 → P3
# ---------------------------------------------------------------------------

def run_full_sequence(robot: Robot, cancel: CancelFlag) -> bool:
    """
    Travel path:

      Start
        │  forward DIST_START_TO_P1_MM
        ▼
       P1  ← pick (turn left, revert)
        │  forward DIST_P1_TO_P2_MM
        ▼
       P2  ← pick (turn left, revert)
        │  forward DIST_P2_TO_P3_MM
        ▼
       P3  ← pick (turn left, revert)

    All legs are forward drives. The robot faces the same direction
    throughout and just steps forward between picks.
    """

    # ---- Drive to P1 and pick ----------------------------------------------
    if cancel.is_set():
        return False
    print("[MOTION] drive forward to P1")
    if not robot.move_forward(
        distance=DIST_START_TO_P1_MM,
        velocity=DRIVE_VELOCITY_MM_S,
        tolerance=DRIVE_TOLERANCE_MM,
        blocking=True,
    ):
        print("[warn] drive to P1 failed")
        robot.stop()
        return False
    
    # Lower from top to pick height before first pick
    robot.step_set_config(
        LIFT_STEPPER,
        max_velocity=LIFT_MAX_VELOCITY,
        acceleration=LIFT_ACCELERATION,
    )
    robot.step_enable(LIFT_STEPPER)
    print("[MANIP] lower to pick height")
    if not lift_move(robot, INIT_LOWER_STEPS):
        print("[warn] lift failed to lower to pick height")
        robot.step_disable(LIFT_STEPPER)
        return False
    robot.step_disable(LIFT_STEPPER)

    if not pick_at_point(robot, "P1", cancel):
        return False

    # ---- Drive forward to P2 and pick --------------------------------------
    if cancel.is_set():
        return False
    print("[MOTION] drive forward to P2")
    if not robot.move_forward(
        distance=DIST_P1_TO_P2_MM,
        velocity=DRIVE_VELOCITY_MM_S,
        tolerance=DRIVE_TOLERANCE_MM,
        blocking=True,
    ):
        print("[warn] drive to P2 failed")
        robot.stop()
        return False

    if not pick_at_point(robot, "P2", cancel):
        return False

    # ---- Drive forward to P3 and pick --------------------------------------
    if cancel.is_set():
        return False
    print("[MOTION] drive forward to P3")
    if not robot.move_forward(
        distance=DIST_P2_TO_P3_MM,
        velocity=DRIVE_VELOCITY_MM_S,
        tolerance=DRIVE_TOLERANCE_MM,
        blocking=True,
    ):
        print("[warn] drive to P3 failed")
        robot.stop()
        return False

    if not pick_at_point(robot, "P3", cancel):
        return False

    robot.stop()
    return True


# ---------------------------------------------------------------------------
# Main FSM loop
# ---------------------------------------------------------------------------

def run(robot: Robot) -> None:
    configure_robot(robot)

    state = "INIT"
    cancel = CancelFlag()
    period = 1.0 / float(DEFAULT_FSM_HZ)
    next_tick = time.monotonic()

    while True:

        if state == "INIT":
            start_robot(robot)
            robot.reset_odometry()
            if not robot.wait_for_odometry_reset(timeout=2.0):
                print("[warn] odometry reset not confirmed; continuing")
                robot.wait_for_pose_update(timeout=0.5)
            show_idle_leds(robot)
            print("[FSM] IDLE — assuming gripper plate at top")
            print("            press BTN_1 to start  |  BTN_2 to cancel mid-run")
            print(f"[CFG] pick order: P1 → P2 → P3  (always turn left to pick)")
            print(f"[CFG] side offset: {PICK_SIDE_OFFSET_MM:.0f} mm")
            print(f"[CFG] distances: start→P1={DIST_START_TO_P1_MM:.0f} mm  "
                  f"P1→P2={DIST_P1_TO_P2_MM:.0f} mm  P2→P3={DIST_P2_TO_P3_MM:.0f} mm")
            print(f"[CFG] gripper open={GRIPPER_OPEN_DEG:.0f}°  close={GRIPPER_CLOSE_DEG:.0f}°")
            print(f"[CFG] lift steps={LIFT_UP_STEPS}  max_vel={LIFT_MAX_VELOCITY} steps/s")
            state = "IDLE"

        elif state == "IDLE":
            if robot.was_button_pressed(Button.BTN_1):
                show_running_leds(robot)
                robot.reset_odometry()
                if not robot.wait_for_odometry_reset(timeout=2.0):
                    print("[warn] odometry reset not confirmed; continuing")
                    robot.wait_for_pose_update(timeout=0.5)
                cancel = CancelFlag()   # fresh flag for this run
                print("[FSM] RUNNING full pick sequence — press BTN_2 to cancel")
                state = "RUN_SEQUENCE"

        elif state == "RUN_SEQUENCE":
            if robot.was_button_pressed(Button.BTN_2):
                cancel.cancel()
                robot.stop()
                robot.step_disable(LIFT_STEPPER)
                robot.disable_servo(GRIPPER_SERVO)
                show_idle_leds(robot)
                print("[FSM] IDLE — sequence cancelled by BTN_2")
                state = "IDLE"
            else:
                ok = run_full_sequence(robot, cancel)
                robot.stop()
                robot.step_disable(LIFT_STEPPER)
                robot.disable_servo(GRIPPER_SERVO)
                show_idle_leds(robot)
                if ok:
                    print("[FSM] SEQUENCE COMPLETE — press BTN_1 to run again")
                    print(" ")
                    state = "IDLE"
                else:
                    print("[FSM] SEQUENCE FAILED — manually lower the platform,")
                    print("      then the robot will return to IDLE")
                    state = "INIT"

        next_tick += period
        sleep_s = next_tick - time.monotonic()
        if sleep_s > 0.0:
            time.sleep(sleep_s)
        else:
            next_tick = time.monotonic()