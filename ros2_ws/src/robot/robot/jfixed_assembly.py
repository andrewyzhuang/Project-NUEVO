"""
pick_sequence.py — integrated motion + manipulation pick sequence
=================================================================
Hardware:
  - Stepper 1  : vertical lift (no limit switch, start platform at bottom)
  - Servo 1    : gripper jaw

Layout:
  Three pick points (P1, P2, P3) are arranged in a line.
  The robot starts facing toward the line (forward = toward the points).
  All pick items sit on the SAME side of the line in world coordinates
  (set WORLD_PICK_SIDE = "left" or "right" relative to the direction the
  robot travels along the line when going from P1 → P3).

Pick order: P1 → P2 → P3

Sequence per point:
  1. Drive forward to the point
  2. Turn 90° to face the pick item (direction computed from heading)
  3. Move sideways offset toward item
  4. Open gripper → lower platform → close gripper → raise platform
  5. Move back to the line
  6. Turn to face the next destination

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
# Distance from start to P1 (first point visited)
DIST_START_TO_P1_MM  = 500.0        # mm — placeholder
# Distance from P1 to P2 (robot drives forward after P1 pick)
DIST_P1_TO_P2_MM     = 500.0        # mm — placeholder
# Distance from P2 to P3 (robot drives forward after P2 pick)
DIST_P2_TO_P3_MM     = 500.0        # mm — placeholder
# How far to move sideways toward the pick item at each point
PICK_SIDE_OFFSET_MM  = 150.0        # mm — placeholder

# Which side of the travel line the pick items are on, in world coordinates.
# "left"  = to the left  when travelling from P1 → P3
# "right" = to the right when travelling from P1 → P3
WORLD_PICK_SIDE = "left"


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
# Heading helpers
# ---------------------------------------------------------------------------

def get_heading_deg(robot: Robot) -> float:
    """Return current odometry heading in degrees."""
    _, _, theta = robot.get_odometry_pose()
    return theta


def pick_turn_deg(heading_deg: float) -> float:
    """
    Return the turn delta (degrees) needed so the robot faces the pick side.

    WORLD_PICK_SIDE is defined relative to the P1→P3 travel direction.
    When the robot is heading toward P3 (positive direction, ~0°) a left
    pick side means turn +90°. When the robot is heading back toward P1
    (~180°) the same world-left item is now on the robot's right, so we
    turn -90°. The sign flips automatically from the dot-product logic below.
    """
    # Normalise heading to (-180, 180]
    h = (heading_deg + 180.0) % 360.0 - 180.0

    # "Forward along the line toward P3" is approximately 0° in our odometry
    # frame (robot starts facing the line, drives straight to P2 first).
    # Dot product of current heading with the P1→P3 direction tells us if the
    # robot is broadly going forward (+) or backward (−) along the line.
    import math
    forward_component = math.cos(math.radians(h))   # +1 = facing P3, -1 = facing P1

    if WORLD_PICK_SIDE == "left":
        # Facing P3 → turn left (+90°); facing P1 → turn right (−90°)
        return 90.0 if forward_component >= 0 else -90.0
    else:
        # Mirror
        return -90.0 if forward_component >= 0 else 90.0


def return_turn_deg(pick_turn: float) -> float:
    """Opposite of the pick turn to face back along the line."""
    return -pick_turn


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


def raise_platform(robot: Robot) -> bool:
    """Raise the lift to the top position from the bottom."""
    robot.step_set_config(
        LIFT_STEPPER,
        max_velocity=LIFT_MAX_VELOCITY,
        acceleration=LIFT_ACCELERATION,
    )
    robot.step_enable(LIFT_STEPPER)
    print("[MANIP] raise platform (pre-pick)")
    ok = lift_move(robot, LIFT_UP_STEPS)
    if not ok:
        print("[warn] lift failed to raise in pre-pick")
        robot.step_disable(LIFT_STEPPER)
        return False
    robot.step_disable(LIFT_STEPPER)
    return True


# ---------------------------------------------------------------------------
# Per-point pick routine
# ---------------------------------------------------------------------------

def pick_at_point(robot: Robot, label: str, approach_dist_mm: float) -> bool:
    """
    Drive to a pick point, perform the pick, and stop ready for the next move.

    approach_dist_mm : distance to drive forward to reach the point.
                       Pass a negative value to drive backward.

    Returns False on any motion or actuator failure.
    """
    print(f"[MOTION] drive to {label}  ({approach_dist_mm:+.0f} mm)")

    if approach_dist_mm >= 0:
        ok = robot.move_forward(
            distance=approach_dist_mm,
            velocity=DRIVE_VELOCITY_MM_S,
            tolerance=DRIVE_TOLERANCE_MM,
            blocking=True,
        )
    else:
        ok = robot.move_backward(
            distance=abs(approach_dist_mm),
            velocity=DRIVE_VELOCITY_MM_S,
            tolerance=DRIVE_TOLERANCE_MM,
            blocking=True,
        )

    if not ok:
        print(f"[warn] drive to {label} failed")
        robot.stop()
        return False

    # Raise platform before turning so the gripper clears the floor
    if not raise_platform(robot):
        return False

    # Turn to face the pick item
    heading  = get_heading_deg(robot)
    turn_deg = pick_turn_deg(heading)
    print(f"[MOTION] turn {turn_deg:+.0f}° to face pick side (heading was {heading:.1f}°)")
    robot.turn_by(
        delta_deg=turn_deg,
        blocking=True,
        tolerance_deg=TURN_TOLERANCE_DEG,
    )

    # Nudge toward the item
    print(f"[MOTION] move {PICK_SIDE_OFFSET_MM:.0f} mm toward item")
    robot.move_forward(
        distance=PICK_SIDE_OFFSET_MM,
        velocity=DRIVE_VELOCITY_MM_S,
        tolerance=DRIVE_TOLERANCE_MM,
        blocking=True,
    )

    # Perform the manipulation (open → lower → close → raise)
    if not perform_pick(robot):
        return False

    # Back up to the line
    print(f"[MOTION] return {PICK_SIDE_OFFSET_MM:.0f} mm to line")
    robot.move_backward(
        distance=PICK_SIDE_OFFSET_MM,
        velocity=DRIVE_VELOCITY_MM_S,
        tolerance=DRIVE_TOLERANCE_MM,
        blocking=True,
    )

    # Turn back to face along the line
    ret_turn = return_turn_deg(turn_deg)
    print(f"[MOTION] turn {ret_turn:+.0f}° to face along line")
    robot.turn_by(
        delta_deg=ret_turn,
        blocking=True,
        tolerance_deg=TURN_TOLERANCE_DEG,
    )

    print(f"[MOTION] {label} pick complete")
    return True


# ---------------------------------------------------------------------------
# Full pick sequence:  P1 → P2 → P3
# ---------------------------------------------------------------------------

def run_full_sequence(robot: Robot) -> bool:
    """
    Execute picks at P1, P2, then P3 in that order.

    Travel path (odometry frame, robot starts at origin facing +X):

      Start ──(forward DIST_START_TO_P1)──► P1
             ──(forward DIST_P1_TO_P2)────► P2
             ──(forward DIST_P2_TO_P3)────► P3

    After each pick the robot is re-oriented along the line in the
    direction needed for the next leg.
    """

    # ---- Pick P1 (first stop, drive forward from start) --------------------
    if not pick_at_point(robot, "P1", DIST_START_TO_P1_MM):
        return False

    # After P1 pick the robot faces toward P3 (forward direction).
    # P2 is ahead of us in the same direction, so drive forward.
    # ---- Pick P2 (between P1 and P3, drive forward) ------------------------
    if not pick_at_point(robot, "P2", DIST_P1_TO_P2_MM):
        return False

    # After P2 pick the robot faces toward P3 (forward direction, ~0°).
    # P3 is ahead of us in the same direction, so drive forward.
    # ---- Pick P3 (far end, drive forward from P2) --------------------------
    if not pick_at_point(robot, "P3", DIST_P2_TO_P3_MM):
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
            print("[FSM] IDLE — manually lower the platform to its bottom position")
            print("            then press BTN_1 to start the pick sequence")
            print(f"[CFG] pick order: P1 → P2 → P3")
            print(f"[CFG] pick side (world): {WORLD_PICK_SIDE}")
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
                print("[FSM] RUNNING full pick sequence")
                state = "RUN_SEQUENCE"

        elif state == "RUN_SEQUENCE":
            ok = run_full_sequence(robot)
            show_idle_leds(robot)
            if ok:
                print("[FSM] SEQUENCE COMPLETE — press BTN_1 to run again")
                print("      (manually lower the platform before re-running)")
                state = "IDLE"
            else:
                # Return to INIT so the operator must manually re-home the
                # platform before the next attempt
                print("[FSM] SEQUENCE FAILED — manually lower the platform,")
                print("      then the robot will return to IDLE")
                state = "INIT"

        next_tick += period
        sleep_s = next_tick - time.monotonic()
        if sleep_s > 0.0:
            time.sleep(sleep_s)
        else:
            next_tick = time.monotonic()