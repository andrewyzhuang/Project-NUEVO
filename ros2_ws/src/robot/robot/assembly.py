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

Pick order: P2 → P1 → P3

Sequence:
  Start → forward to P2 → turn left → pick → turn right (revert) →
  backward to P1 → turn left → pick → turn right (revert) →
  forward to P3 → turn left → pick → turn right (revert) → done

HOW TO RUN
----------
    cp examples/pick_sequence.py main.py
    ros2 run robot robot

Press BTN_1 to start. Manually lower the platform to its bottom before
pressing BTN_1. On any actuator failure the robot stops and returns to
INIT so you can re-home the platform by hand.
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
GRIPPER_OPEN_DEG    = 15.0          # degrees — jaw open
GRIPPER_CLOSE_DEG   = 120.0         # degrees — jaw closed
GRIPPER_SETTLE_S    = 1.0           # seconds to wait after each servo move

# --- Lift (Stepper 1) ---
LIFT_STEPPER        = Stepper.STEPPER_1
LIFT_UP_STEPS       = 2000          # steps from bottom to top
LIFT_MAX_VELOCITY   = 800           # steps/s
LIFT_ACCELERATION   = 400           # steps/s²
LIFT_MOVE_TIMEOUT_S = 10.0          # max seconds per lift move

# --- Drive ---
DRIVE_VELOCITY_MM_S  = 200.0        # mm/s for all forward/backward moves
DRIVE_TOLERANCE_MM   = 20.0         # mm position tolerance
TURN_TOLERANCE_DEG   = 3.0          # degrees heading tolerance

# --- Point geometry (set these to match your actual field) ---
# Distance from start to P2 (first point visited)
DIST_START_TO_P2_MM  = 1000.0       # mm — placeholder
# Distance from P2 backward to P1
DIST_P2_TO_P1_MM     = 500.0        # mm — placeholder
# Distance from P1 forward to P3 (past P2 to the far end)
DIST_P1_TO_P3_MM     = 1000.0       # mm — placeholder
# How far to move toward the pick item at each point
PICK_SIDE_OFFSET_MM  = 150.0        # mm — placeholder


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


def raise_platform(robot: Robot) -> bool:
    """Raise the lift to the top position from the bottom."""
    robot.step_set_config(
        LIFT_STEPPER,
        max_velocity=LIFT_MAX_VELOCITY,
        acceleration=LIFT_ACCELERATION,
    )
    robot.step_enable(LIFT_STEPPER)
    print("[MANIP] raise platform")
    ok = lift_move(robot, LIFT_UP_STEPS)
    if not ok:
        print("[warn] lift failed to raise")
        robot.step_disable(LIFT_STEPPER)
        return False
    robot.step_disable(LIFT_STEPPER)
    return True


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
    if not lift_move(robot, -LIFT_UP_STEPS):
        print("[warn] lift failed to lower")
        robot.step_disable(LIFT_STEPPER)
        robot.disable_servo(GRIPPER_SERVO)
        return False

    print("[MANIP] close gripper")
    robot.set_servo(GRIPPER_SERVO, GRIPPER_CLOSE_DEG)
    time.sleep(GRIPPER_SETTLE_S)

    print("[MANIP] raise platform")
    if not lift_move(robot, LIFT_UP_STEPS):
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

def pick_at_point(robot: Robot, label: str) -> bool:
    """
    At the current position:
      1. Raise platform
      2. Turn left 90°
      3. Move forward PICK_SIDE_OFFSET_MM toward item
      4. Perform pick (open → lower → close → raise)
      5. Move backward PICK_SIDE_OFFSET_MM back to line
      6. Turn right 90° (revert left turn)

    The robot must already be positioned at the point and facing
    along the line before this is called.

    Returns False on any failure.
    """

    # Always turn left to face the pick item
    print(f"[MOTION] {label} — turn left 90°")
    robot.turn_by(
        delta_deg=90.0,
        blocking=True,
        tolerance_deg=TURN_TOLERANCE_DEG,
    )

    # Nudge toward the item
    print(f"[MOTION] {label} — move {PICK_SIDE_OFFSET_MM:.0f} mm toward item")
    robot.move_forward(
        distance=PICK_SIDE_OFFSET_MM,
        velocity=DRIVE_VELOCITY_MM_S,
        tolerance=DRIVE_TOLERANCE_MM,
        blocking=True,
    )

    # Perform the manipulation
    if not perform_pick(robot):
        return False

    # Back up to the line
    print(f"[MOTION] {label} — return {PICK_SIDE_OFFSET_MM:.0f} mm to line")
    robot.move_backward(
        distance=PICK_SIDE_OFFSET_MM,
        velocity=DRIVE_VELOCITY_MM_S,
        tolerance=DRIVE_TOLERANCE_MM,
        blocking=True,
    )

    # Revert the left turn (turn right 90°) to face along the line again
    print(f"[MOTION] {label} — turn right 90° (revert)")
    robot.turn_by(
        delta_deg=-90.0,
        blocking=True,
        tolerance_deg=TURN_TOLERANCE_DEG,
    )

    print(f"[MOTION] {label} pick complete")
    return True


# ---------------------------------------------------------------------------
# Full pick sequence:  P2 → P1 → P3
# ---------------------------------------------------------------------------

def run_full_sequence(robot: Robot) -> bool:
    """
    Travel path:

      Start
        │  forward DIST_START_TO_P2_MM
        ▼
       P2  ← pick (turn left, revert)
        │  backward DIST_P2_TO_P1_MM
        ▼
       P1  ← pick (turn left, revert)
        │  forward DIST_P1_TO_P3_MM
        ▼
       P3  ← pick (turn left, revert)

    After each pick the robot is facing along the line in the same
    direction it arrived, ready for the next drive leg.

    NOTE: after the P1 pick the robot is still facing toward P1
    (it arrived backward). A 180° turn is needed before driving
    forward to P3.
    """

    # ---- Drive to P2 and pick ----------------------------------------------
    print("[MOTION] drive forward to P2")
    if not robot.move_forward(
        distance=DIST_START_TO_P2_MM,
        velocity=DRIVE_VELOCITY_MM_S,
        tolerance=DRIVE_TOLERANCE_MM,
        blocking=True,
    ):
        print("[warn] drive to P2 failed")
        robot.stop()
        return False
    
    # remove this comment when traffic light height is determined
    #if not raise_platform(robot):
    #    return False

    if not pick_at_point(robot, "P2"):
        return False

    # ---- Drive backward to P1 and pick -------------------------------------
    # After the P2 pick the robot faces toward P3 (the direction it arrived).
    # Drive backward to reach P1.
    print("[MOTION] drive backward to P1")
    if not robot.move_backward(
        distance=DIST_P2_TO_P1_MM,
        velocity=DRIVE_VELOCITY_MM_S,
        tolerance=DRIVE_TOLERANCE_MM,
        blocking=True,
    ):
        print("[warn] drive to P1 failed")
        robot.stop()
        return False

    if not pick_at_point(robot, "P1"):
        return False

    # ---- Turn 180° to face P3 then drive forward ---------------------------
    # After the P1 pick the robot is still facing toward P3 (same heading as
    # after P2 pick, since pick_at_point always restores the heading).
    # P1 is behind the robot so we just drive forward toward P3.
    print("[MOTION] drive forward to P3")
    if not robot.move_forward(
        distance=DIST_P1_TO_P3_MM,
        velocity=DRIVE_VELOCITY_MM_S,
        tolerance=DRIVE_TOLERANCE_MM,
        blocking=True,
    ):
        print("[warn] drive to P3 failed")
        robot.stop()
        return False

    if not pick_at_point(robot, "P3"):
        return False

    robot.stop()
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
            robot.reset_odometry()
            if not robot.wait_for_odometry_reset(timeout=2.0):
                print("[warn] odometry reset not confirmed; continuing")
                robot.wait_for_pose_update(timeout=0.5)
            show_idle_leds(robot)
            print("[FSM] IDLE — assuming gripper plate at top")
            print("            then press BTN_1 to start the pick sequence")
            print(f"[CFG] pick order: P2 → P1 → P3  (always turn left to pick)")
            print(f"[CFG] side offset: {PICK_SIDE_OFFSET_MM:.0f} mm")
            print(f"[CFG] distances: start→P2={DIST_START_TO_P2_MM:.0f} mm  "
                  f"P2→P1={DIST_P2_TO_P1_MM:.0f} mm  P1→P3={DIST_P1_TO_P3_MM:.0f} mm")
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
                print("[FSM] RUNNING full pick sequence")
                state = "RUN_SEQUENCE"

        elif state == "RUN_SEQUENCE":
            ok = run_full_sequence(robot)
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