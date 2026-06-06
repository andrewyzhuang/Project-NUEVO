"""
=================================================================
Hardware:
  - Stepper 1  : vertical lift (no limit switch, platform starts at top)
  - Servo 1    : gripper jaw

Layout:
  Three pick points (P1, P2, P3) are arranged in a line.
  The robot starts facing toward the line (forward = toward the points).
  All picks use a hardcoded LEFT turn to face the item.
  Platform starts at the top (HEIGHT_1) and descends to pick.

Platform heights (all measured as steps down from HEIGHT_1):
  HEIGHT_1 : starting position — top of platform lift
  HEIGHT_2 : table/platform surface level
  HEIGHT_3 : height of one patty on the table/platform
  HEIGHT_4 : height of one bun on the table/platform
  HEIGHT_5 : clearance height — slightly above HEIGHT_4

Pick order: P1 → P2 → P3

Sequence per point:
  P1: forward → turn left → step forward → open gripper →
      lower to HEIGHT_2 → close gripper → raise to HEIGHT_5 →
      step back → turn right

  P2: forward → turn left → step forward →
      lower to HEIGHT_3 → open gripper → lower to HEIGHT_2 →
      close gripper → raise to HEIGHT_5 →
      step back → turn right

  P3: forward → turn left → step forward →
      lower to HEIGHT_4 → open gripper → lower to HEIGHT_2 →
      close gripper → raise to HEIGHT_5 →
      step back → turn right

  End: raise platform back to HEIGHT_1

HOW TO RUN
----------
    cp assembly.py main.py
    ros2 run robot robot

Press BTN_1 to start. On any actuator failure the robot stops and returns
to INIT. BTN_2 immediately terminates the program at any point.
"""

from __future__ import annotations

import sys
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
GRIPPER_OPEN_DEG    = 60            # degrees — jaw open
GRIPPER_CLOSE_DEG   = 160.0         # degrees — jaw closed (prev: 170)
GRIPPER_SETTLE_S    = 1.0           # seconds to wait after each servo move

# --- Lift (Stepper 1) ---
# All heights are defined as steps DOWN from HEIGHT_1 (top/start position).
# Positive step values = downward movement from starting position.
LIFT_STEPPER        = Stepper.STEPPER_1
LIFT_MAX_VELOCITY   = 800           # steps/s
LIFT_ACCELERATION   = 400           # steps/s²
LIFT_MOVE_TIMEOUT_S = 10.0          # max seconds per lift move

# Platform height definitions (steps below HEIGHT_1)
HEIGHT_1_STEPS = 0       # starting position — top of lift travel
HEIGHT_2_STEPS = 2475    # table/platform surface level (prev: 2100)
HEIGHT_3_STEPS = 1875    # one patty height above table (prev: 1200)
HEIGHT_4_STEPS = 1875    # one bun height above table — set to match your build
HEIGHT_5_STEPS = 2000    # clearance height — slightly above HEIGHT_4

# --- Drive ---
DRIVE_VELOCITY_MM_S  = 75.0         # mm/s for all forward/backward moves (prev: 100)
DRIVE_TOLERANCE_MM   = 20.0         # mm position tolerance
TURN_TOLERANCE_DEG   = 3.0          # degrees heading tolerance

# --- Point geometry (set these to match your actual field) ---
DIST_START_TO_P1_MM  = 1150.0       # distance from start to P1 (prev: 950)
DIST_P1_TO_P2_MM     = 150.0        # distance from P1 to P2
DIST_P2_TO_P3_MM     = 150.0        # distance from P2 to P3
PICK_SIDE_OFFSET_MM  = 50.0          # distance to step toward item before picking


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
# Lift primitives
# ---------------------------------------------------------------------------

def lift_move(robot: Robot, steps: int) -> bool:
    """Move the lift by `steps` relative to current position.
    Positive = downward, negative = upward."""
    return robot.step_move(
        LIFT_STEPPER,
        steps=steps,
        move_type=StepMoveType.RELATIVE,
        blocking=True,
        timeout=LIFT_MOVE_TIMEOUT_S,
    )


def lift_enable(robot: Robot) -> None:
    """Configure and enable the lift stepper."""
    robot.step_set_config(
        LIFT_STEPPER,
        max_velocity=LIFT_MAX_VELOCITY,
        acceleration=LIFT_ACCELERATION,
    )
    robot.step_enable(LIFT_STEPPER)


def lift_to_height(robot: Robot, from_steps: int, to_steps: int) -> bool:
    """
    Move the platform from one absolute height (in steps below HEIGHT_1)
    to another. Positive delta = downward, negative delta = upward.
    Returns False on failure.
    """
    delta = to_steps - from_steps
    if delta == 0:
        return True
    direction = "down" if delta > 0 else "up"
    print(f"[MANIP] move platform {direction} by {abs(delta)} steps")
    return lift_move(robot, delta)


# ---------------------------------------------------------------------------
# Per-point pick routines
# ---------------------------------------------------------------------------

def pick_p1(robot: Robot, cancel: CancelFlag, current_height: int) -> tuple[bool, int]:
    """
    P1 pick — platform is already at HEIGHT_5 (clearance) on entry.

    Sequence:
      Turn left 90° → step forward → open gripper →
      lower to HEIGHT_2 (table surface) → close gripper →
      raise to HEIGHT_5 (clearance) → step back → turn right 90°

    Returns (success, new_current_height_steps).
    """
    label = "P1"

    if cancel.is_set():
        return False, current_height

    print(f"[MOTION] {label} — turn left 90°")
    robot.turn_by(delta_deg=90.0, blocking=True, tolerance_deg=TURN_TOLERANCE_DEG)

    if cancel.is_set():
        return False, current_height

    print(f"[MOTION] {label} — step {PICK_SIDE_OFFSET_MM:.0f} mm toward item")
    robot.move_forward(
        distance=PICK_SIDE_OFFSET_MM,
        velocity=DRIVE_VELOCITY_MM_S,
        tolerance=DRIVE_TOLERANCE_MM,
        blocking=True,
    )

    if cancel.is_set():
        return False, current_height

    # Open gripper at clearance height before descending
    robot.enable_servo(GRIPPER_SERVO)
    print(f"[MANIP] {label} — open gripper")
    robot.set_servo(GRIPPER_SERVO, GRIPPER_OPEN_DEG)
    time.sleep(GRIPPER_SETTLE_S)

    # Lower platform from HEIGHT_5 down to HEIGHT_2 (table surface)
    lift_enable(robot)
    print(f"[MANIP] {label} — lower platform to table surface (HEIGHT_2)")
    if not lift_to_height(robot, current_height, HEIGHT_2_STEPS):
        print(f"[warn] {label} — lift failed to reach table surface")
        robot.step_disable(LIFT_STEPPER)
        robot.disable_servo(GRIPPER_SERVO)
        return False, current_height
    current_height = HEIGHT_2_STEPS

    print(f"[MANIP] {label} — close gripper")
    robot.set_servo(GRIPPER_SERVO, GRIPPER_CLOSE_DEG)
    time.sleep(GRIPPER_SETTLE_S)

    # Raise platform up to HEIGHT_5 (clearance)
    print(f"[MANIP] {label} — raise platform to clearance height (HEIGHT_5)")
    if not lift_to_height(robot, current_height, HEIGHT_5_STEPS):
        print(f"[warn] {label} — lift failed to reach clearance height")
        robot.step_disable(LIFT_STEPPER)
        robot.disable_servo(GRIPPER_SERVO)
        return False, current_height
    current_height = HEIGHT_5_STEPS

    robot.step_disable(LIFT_STEPPER)
    robot.disable_servo(GRIPPER_SERVO)

    if cancel.is_set():
        return False, current_height

    print(f"[MOTION] {label} — step back {PICK_SIDE_OFFSET_MM:.0f} mm to line")
    robot.move_backward(
        distance=PICK_SIDE_OFFSET_MM,
        velocity=DRIVE_VELOCITY_MM_S,
        tolerance=DRIVE_TOLERANCE_MM,
        blocking=True,
    )

    if cancel.is_set():
        return False, current_height

    print(f"[MOTION] {label} — turn right 90° (revert)")
    robot.turn_by(delta_deg=-90.0, blocking=True, tolerance_deg=TURN_TOLERANCE_DEG)

    print(f"[MOTION] {label} pick complete")
    return True, current_height


def pick_p2(robot: Robot, cancel: CancelFlag, current_height: int) -> tuple[bool, int]:
    """
    P2 pick — platform is at HEIGHT_5 (clearance) on entry.

    Sequence:
      Turn left 90° → step forward →
      lower to HEIGHT_3 (patty height) → open gripper →
      lower to HEIGHT_2 (table surface) → close gripper →
      raise to HEIGHT_5 (clearance) → step back → turn right 90°

    Returns (success, new_current_height_steps).
    """
    label = "P2"

    if cancel.is_set():
        return False, current_height

    print(f"[MOTION] {label} — turn left 90°")
    robot.turn_by(delta_deg=90.0, blocking=True, tolerance_deg=TURN_TOLERANCE_DEG)

    if cancel.is_set():
        return False, current_height

    print(f"[MOTION] {label} — step {PICK_SIDE_OFFSET_MM:.0f} mm toward item")
    robot.move_forward(
        distance=PICK_SIDE_OFFSET_MM,
        velocity=DRIVE_VELOCITY_MM_S,
        tolerance=DRIVE_TOLERANCE_MM,
        blocking=True,
    )

    if cancel.is_set():
        return False, current_height

    lift_enable(robot)

    # Lower platform from HEIGHT_5 down to HEIGHT_3 (patty height) — gripper still open
    print(f"[MANIP] {label} — lower platform to patty height (HEIGHT_3)")
    if not lift_to_height(robot, current_height, HEIGHT_3_STEPS):
        print(f"[warn] {label} — lift failed to reach patty height")
        robot.step_disable(LIFT_STEPPER)
        return False, current_height
    current_height = HEIGHT_3_STEPS

    # Open gripper at patty height before descending to table
    robot.enable_servo(GRIPPER_SERVO)
    print(f"[MANIP] {label} — open gripper at patty height")
    robot.set_servo(GRIPPER_SERVO, GRIPPER_OPEN_DEG)
    time.sleep(GRIPPER_SETTLE_S)

    # Lower further from HEIGHT_3 down to HEIGHT_2 (table surface)
    print(f"[MANIP] {label} — lower platform to table surface (HEIGHT_2)")
    if not lift_to_height(robot, current_height, HEIGHT_2_STEPS):
        print(f"[warn] {label} — lift failed to reach table surface")
        robot.step_disable(LIFT_STEPPER)
        robot.disable_servo(GRIPPER_SERVO)
        return False, current_height
    current_height = HEIGHT_2_STEPS

    print(f"[MANIP] {label} — close gripper")
    robot.set_servo(GRIPPER_SERVO, GRIPPER_CLOSE_DEG)
    time.sleep(GRIPPER_SETTLE_S)

    # Raise platform up to HEIGHT_5 (clearance)
    print(f"[MANIP] {label} — raise platform to clearance height (HEIGHT_5)")
    if not lift_to_height(robot, current_height, HEIGHT_5_STEPS):
        print(f"[warn] {label} — lift failed to reach clearance height")
        robot.step_disable(LIFT_STEPPER)
        robot.disable_servo(GRIPPER_SERVO)
        return False, current_height
    current_height = HEIGHT_5_STEPS

    robot.step_disable(LIFT_STEPPER)
    robot.disable_servo(GRIPPER_SERVO)

    if cancel.is_set():
        return False, current_height

    print(f"[MOTION] {label} — step back {PICK_SIDE_OFFSET_MM:.0f} mm to line")
    robot.move_backward(
        distance=PICK_SIDE_OFFSET_MM,
        velocity=DRIVE_VELOCITY_MM_S,
        tolerance=DRIVE_TOLERANCE_MM,
        blocking=True,
    )

    if cancel.is_set():
        return False, current_height

    print(f"[MOTION] {label} — turn right 90° (revert)")
    robot.turn_by(delta_deg=-90.0, blocking=True, tolerance_deg=TURN_TOLERANCE_DEG)

    print(f"[MOTION] {label} pick complete")
    return True, current_height


def pick_p3(robot: Robot, cancel: CancelFlag, current_height: int) -> tuple[bool, int]:
    """
    P3 pick — platform is at HEIGHT_5 (clearance) on entry.

    Sequence:
      Turn left 90° → step forward →
      lower to HEIGHT_4 (bun height) → open gripper →
      lower to HEIGHT_2 (table surface) → close gripper →
      raise to HEIGHT_5 (clearance) → step back → turn right 90°

    Returns (success, new_current_height_steps).
    """
    label = "P3"

    if cancel.is_set():
        return False, current_height

    print(f"[MOTION] {label} — turn left 90°")
    robot.turn_by(delta_deg=90.0, blocking=True, tolerance_deg=TURN_TOLERANCE_DEG)

    if cancel.is_set():
        return False, current_height

    print(f"[MOTION] {label} — step {PICK_SIDE_OFFSET_MM:.0f} mm toward item")
    robot.move_forward(
        distance=PICK_SIDE_OFFSET_MM,
        velocity=DRIVE_VELOCITY_MM_S,
        tolerance=DRIVE_TOLERANCE_MM,
        blocking=True,
    )

    if cancel.is_set():
        return False, current_height

    lift_enable(robot)

    # Lower platform from HEIGHT_5 down to HEIGHT_4 (bun height) — gripper still open
    print(f"[MANIP] {label} — lower platform to bun height (HEIGHT_4)")
    if not lift_to_height(robot, current_height, HEIGHT_4_STEPS):
        print(f"[warn] {label} — lift failed to reach bun height")
        robot.step_disable(LIFT_STEPPER)
        return False, current_height
    current_height = HEIGHT_4_STEPS

    # Open gripper at bun height before descending to table
    robot.enable_servo(GRIPPER_SERVO)
    print(f"[MANIP] {label} — open gripper at bun height")
    robot.set_servo(GRIPPER_SERVO, GRIPPER_OPEN_DEG)
    time.sleep(GRIPPER_SETTLE_S)

    # Lower further from HEIGHT_4 down to HEIGHT_2 (table surface)
    print(f"[MANIP] {label} — lower platform to table surface (HEIGHT_2)")
    if not lift_to_height(robot, current_height, HEIGHT_2_STEPS):
        print(f"[warn] {label} — lift failed to reach table surface")
        robot.step_disable(LIFT_STEPPER)
        robot.disable_servo(GRIPPER_SERVO)
        return False, current_height
    current_height = HEIGHT_2_STEPS

    print(f"[MANIP] {label} — close gripper")
    robot.set_servo(GRIPPER_SERVO, GRIPPER_CLOSE_DEG)
    time.sleep(GRIPPER_SETTLE_S)

    # Raise platform up to HEIGHT_5 (clearance)
    print(f"[MANIP] {label} — raise platform to clearance height (HEIGHT_5)")
    if not lift_to_height(robot, current_height, HEIGHT_5_STEPS):
        print(f"[warn] {label} — lift failed to reach clearance height")
        robot.step_disable(LIFT_STEPPER)
        robot.disable_servo(GRIPPER_SERVO)
        return False, current_height
    current_height = HEIGHT_5_STEPS

    robot.step_disable(LIFT_STEPPER)
    robot.disable_servo(GRIPPER_SERVO)

    if cancel.is_set():
        return False, current_height

    print(f"[MOTION] {label} — step back {PICK_SIDE_OFFSET_MM:.0f} mm to line")
    robot.move_backward(
        distance=PICK_SIDE_OFFSET_MM,
        velocity=DRIVE_VELOCITY_MM_S,
        tolerance=DRIVE_TOLERANCE_MM,
        blocking=True,
    )

    if cancel.is_set():
        return False, current_height

    print(f"[MOTION] {label} — turn right 90° (revert)")
    robot.turn_by(delta_deg=-90.0, blocking=True, tolerance_deg=TURN_TOLERANCE_DEG)

    print(f"[MOTION] {label} pick complete")
    return True, current_height


# ---------------------------------------------------------------------------
# Full pick sequence:  P1 → P2 → P3
# ---------------------------------------------------------------------------

def run_full_sequence(robot: Robot, cancel: CancelFlag) -> bool:
    """
    Platform starts at HEIGHT_1 (top). Before the first pick it descends
    to HEIGHT_5 (clearance), then each pick routine handles its own
    descent/ascent. After all picks the platform returns to HEIGHT_1.

    Travel path:

      Start (platform at HEIGHT_1)
        │  forward DIST_START_TO_P1_MM
        │  lower platform to HEIGHT_5
        ▼
       P1  ← open gripper, lower to HEIGHT_2, close, raise to HEIGHT_5
        │  forward DIST_P1_TO_P2_MM
        ▼
       P2  ← lower to HEIGHT_3, open gripper, lower to HEIGHT_2, close, raise to HEIGHT_5
        │  forward DIST_P2_TO_P3_MM
        ▼
       P3  ← lower to HEIGHT_4, open gripper, lower to HEIGHT_2, close, raise to HEIGHT_5
        │
        ▼  raise platform back to HEIGHT_1 (starting position)
    """
    current_height = HEIGHT_1_STEPS  # platform starts at the top

    # ---- Drive to P1 -------------------------------------------------------
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

    # Lower platform from HEIGHT_1 (top) down to HEIGHT_5 (clearance) before first pick
    lift_enable(robot)
    print("[MANIP] lower platform from starting position to clearance height (HEIGHT_5)")
    if not lift_to_height(robot, current_height, HEIGHT_5_STEPS):
        print("[warn] lift failed to reach clearance height")
        robot.step_disable(LIFT_STEPPER)
        return False
    current_height = HEIGHT_5_STEPS
    robot.step_disable(LIFT_STEPPER)

    ok, current_height = pick_p1(robot, cancel, current_height)
    if not ok:
        return False

    # ---- Drive forward to P2 -----------------------------------------------
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

    ok, current_height = pick_p2(robot, cancel, current_height)
    if not ok:
        return False

    # ---- Drive forward to P3 -----------------------------------------------
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

    ok, current_height = pick_p3(robot, cancel, current_height)
    if not ok:
        return False

    # ---- Return platform to starting position (HEIGHT_1 / top) ------------
    lift_enable(robot)
    print("[MANIP] raise platform back to starting position (HEIGHT_1)")
    if not lift_to_height(robot, current_height, HEIGHT_1_STEPS):
        print("[warn] lift failed to return to starting position")
        robot.step_disable(LIFT_STEPPER)
        return False
    robot.step_disable(LIFT_STEPPER)
    print("[MANIP] platform returned to starting position")

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

        # --- BTN_2 pressed anywhere: immediately terminate the program -------
        if robot.was_button_pressed(Button.BTN_2):
            robot.stop()
            robot.step_disable(LIFT_STEPPER)
            robot.disable_servo(GRIPPER_SERVO)
            show_idle_leds(robot)
            print("[FSM] BTN_2 pressed — program terminated")
            sys.exit(0)

        if state == "INIT":
            start_robot(robot)
            robot.reset_odometry()
            if not robot.wait_for_odometry_reset(timeout=2.0):
                print("[warn] odometry reset not confirmed; continuing")
                robot.wait_for_pose_update(timeout=0.5)
            show_idle_leds(robot)
            print("[FSM] IDLE — platform must be at starting position (HEIGHT_1 / top)")
            print("            press BTN_1 to start  |  BTN_2 to terminate program")
            print(f"[CFG] pick order: P1 → P2 → P3  (always turn left to pick)")
            print(f"[CFG] side offset: {PICK_SIDE_OFFSET_MM:.0f} mm")
            print(f"[CFG] distances: start→P1={DIST_START_TO_P1_MM:.0f} mm  "
                  f"P1→P2={DIST_P1_TO_P2_MM:.0f} mm  P2→P3={DIST_P2_TO_P3_MM:.0f} mm")
            print(f"[CFG] gripper open={GRIPPER_OPEN_DEG:.0f}°  close={GRIPPER_CLOSE_DEG:.0f}°")
            print(f"[CFG] HEIGHT_1={HEIGHT_1_STEPS} steps  HEIGHT_2={HEIGHT_2_STEPS} steps  "
                  f"HEIGHT_3={HEIGHT_3_STEPS} steps  HEIGHT_4={HEIGHT_4_STEPS} steps  "
                  f"HEIGHT_5={HEIGHT_5_STEPS} steps")
            print(f"[CFG] lift max_vel={LIFT_MAX_VELOCITY} steps/s")
            state = "IDLE"

        elif state == "IDLE":
            if robot.was_button_pressed(Button.BTN_1):
                show_running_leds(robot)
                robot.reset_odometry()
                if not robot.wait_for_odometry_reset(timeout=2.0):
                    print("[warn] odometry reset not confirmed; continuing")
                    robot.wait_for_pose_update(timeout=0.5)
                cancel = CancelFlag()   # fresh flag for this run
                print("[FSM] RUNNING full pick sequence — press BTN_2 to terminate")
                state = "RUN_SEQUENCE"

        elif state == "RUN_SEQUENCE":
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
                print("[FSM] SEQUENCE FAILED — manually return the platform to HEIGHT_1,")
                print("      then the robot will return to IDLE")
                state = "INIT"

        next_tick += period
        sleep_s = next_tick - time.monotonic()
        if sleep_s > 0.0:
            time.sleep(sleep_s)
        else:
            next_tick = time.monotonic()