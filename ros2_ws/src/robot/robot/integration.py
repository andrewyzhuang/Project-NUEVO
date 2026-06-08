"""
main.py — Full integrated mission sequence
==========================================

Stage overview
--------------
  1. TRAFFIC LIGHT  : turn to scan angle alpha, wait for green, turn back to neutral
  2. ASSEMBLY       : pick sequence P2 → P1 → P3 (stepper lift + servo gripper)
  3. NAVIGATION     : vector-blended pure-pursuit path following
  4. PERSON DETECT  : classify customer_a / customer_b over a confirmation hold period
  5. DELIVER A or B : 90° right turn → drive DIST_A or DIST_B forward
                      → 90° left turn → open gripper
                      → revert left turn → drive (DIST_A/B − RETURN_OFFSET_MM) forward
                      → stop 2 s → drive FINAL_FORWARD_MM forward

HOW TO RUN
----------
    cp main.py main.py
    ros2 run robot robot

Press BTN_1 to start from Stage 1.
On any actuator/drive failure the FSM falls back to INIT so you can
re-home the platform by hand before pressing BTN_1 again.
"""

from __future__ import annotations

import time

from robot.hardware_map import (
    Button,
    DEFAULT_FSM_HZ,
    LED,
    INITIAL_THETA_DEG,
    LIDAR_FOV_DEG,
    LIDAR_MOUNT_THETA_DEG,
    LIDAR_MOUNT_X_MM,
    LIDAR_MOUNT_Y_MM,
    LIDAR_RANGE_MAX_MM,
    LIDAR_RANGE_MIN_MM,
    LEFT_WHEEL_DIR_INVERTED,
    LEFT_WHEEL_MOTOR,
    POSITION_UNIT,
    RIGHT_WHEEL_DIR_INVERTED,
    RIGHT_WHEEL_MOTOR,
    ServoChannel,
    StepMoveType,
    Stepper,
    TAG_BODY_OFFSET_X_MM,
    TAG_BODY_OFFSET_Y_MM,
    WHEEL_BASE,
    WHEEL_DIAMETER,
)
from robot.robot import FirmwareState, Robot


# ===========================================================================
# ── STAGE 1 : Traffic-light scan ───────────────────────────────────────────
# ===========================================================================

# Angle the robot turns to face the traffic light (positive = left/CCW)
TRAFFIC_LIGHT_SCAN_ALPHA_DEG    = 45.0
TRAFFIC_LIGHT_TURN_TOLERANCE_DEG = 3.0
MIN_TRAFFIC_LIGHT_CONFIDENCE    = 0.50
VISION_STALE_SEC                = 3.0


# ===========================================================================
# ── STAGE 2 : Assembly (pick sequence) ─────────────────────────────────────
# ===========================================================================

# Gripper (Servo 1)
GRIPPER_SERVO       = ServoChannel.CH_1
GRIPPER_OPEN_DEG    = 15.0
GRIPPER_CLOSE_DEG   = 120.0
GRIPPER_SETTLE_S    = 1.0

# Lift (Stepper 1)
LIFT_STEPPER        = Stepper.STEPPER_1
LIFT_UP_STEPS       = 2000
LIFT_MAX_VELOCITY   = 800
LIFT_ACCELERATION   = 400
LIFT_MOVE_TIMEOUT_S = 10.0

# Drive shared constants (also used by assembly)
DRIVE_VELOCITY_MM_S  = 200.0
DRIVE_TOLERANCE_MM   = 20.0
TURN_TOLERANCE_DEG   = 3.0

# Pick-point geometry (mm) — adjust to your field
DIST_START_TO_P2_MM  = 1000.0
DIST_P2_TO_P1_MM     =  500.0
DIST_P1_TO_P3_MM     = 1000.0
PICK_SIDE_OFFSET_MM  =  150.0


# ===========================================================================
# ── STAGE 3 : Navigation (vector-blended pure pursuit) ─────────────────────
# ===========================================================================

ENABLE_LIDAR = True
ENABLE_GPS   = False
TAG_ID       = -1

GPS_POSITION_ALPHA              = 0.10
ENABLE_GPS_TANGENT_HEADING      = False
GPS_TANGENT_ALPHA               = 0.15
GPS_TANGENT_MIN_DISPLACEMENT_MM = 200.0

init_x = 0.0
init_y = 0.0

# (X, Y, repulsion_range_mm) — tune x/y to your field; keep repulsion_range constant
PATH_CONTROL_POINTS = [
    (init_x, init_y, 240.0),
    (0.0,    2150.0, 240.0),
    (530.0,  2150.0, 160.0),
    (530.0,  -550.0, 240.0),
    (1300.0, -550.0, 240.0),
    (1300.0, 2100.0, 360.0),
    (1500.0, 2100.0, 240.0),
]

NAV_VELOCITY_MM_S    = 150.0
NAV_LOOKAHEAD_MM     = 150.0
NAV_TOLERANCE_MM     =  25.0
NAV_ADVANCE_RADIUS_MM = 100.0
NAV_MAX_ANGULAR_RAD_S =   0.6
NAV_REPULSION_RANGE_MM = 450.0
NAV_REPULSION_GAIN     = 600.0
NAV_STATUS_INTERVAL_S  =   0.5


# ===========================================================================
# ── STAGE 4 : Person detection ─────────────────────────────────────────────
# ===========================================================================

PERSON_CONFIRM_HOLD_S   = 1.5   # same class must be seen continuously for this long
MIN_PERSON_CONFIDENCE   = 0.50  # ignore detections below this threshold


# ===========================================================================
# ── STAGE 5 : Delivery ─────────────────────────────────────────────────────
# ===========================================================================

# Forward distances after the 90° right turn
DIST_CUSTOMER_A_MM   = 800.0    # placeholder — distance to Customer A
DIST_CUSTOMER_B_MM   = 1400.0   # placeholder — distance to Customer B

# After delivering, the robot reverses this much less than it drove forward
# so it ends up slightly past the pick-side line before the final push
RETURN_OFFSET_MM     = 200.0    # placeholder

# Final forward drive after the 2-second stop
FINAL_FORWARD_MM     = 300.0    # placeholder

DELIVERY_VELOCITY_MM_S  = 200.0
DELIVERY_TOLERANCE_MM   =  20.0


# ===========================================================================
# ── LED helpers ─────────────────────────────────────────────────────────────
# ===========================================================================

def dim_all_leds(robot: Robot) -> None:
    for led in (LED.RED, LED.GREEN, LED.BLUE, LED.ORANGE, LED.PURPLE):
        robot.set_led(led, 0)


def show_idle_leds(robot: Robot) -> None:
    robot.set_led(LED.ORANGE, 200)
    robot.set_led(LED.GREEN, 0)


def show_running_leds(robot: Robot) -> None:
    robot.set_led(LED.ORANGE, 0)
    robot.set_led(LED.GREEN, 200)


# ===========================================================================
# ── Robot startup helpers ───────────────────────────────────────────────────
# ===========================================================================

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
    robot.enable_vision()

    if ENABLE_LIDAR:
        robot.enable_lidar()
        robot.set_lidar_mount(
            x_mm=LIDAR_MOUNT_X_MM,
            y_mm=LIDAR_MOUNT_Y_MM,
            theta_deg=LIDAR_MOUNT_THETA_DEG,
        )
        robot.set_lidar_filter(
            range_min_mm=LIDAR_RANGE_MIN_MM,
            range_max_mm=LIDAR_RANGE_MAX_MM,
            fov_deg=LIDAR_FOV_DEG,
        )
        robot.start_lidar_world_publisher()

    if ENABLE_GPS:
        robot.enable_gps()
        robot.set_tracked_tag_id(TAG_ID)
        robot.set_tag_body_offset(TAG_BODY_OFFSET_X_MM, TAG_BODY_OFFSET_Y_MM)
        robot.set_position_fusion_alpha(GPS_POSITION_ALPHA)


def start_robot(robot: Robot) -> None:
    current = robot.get_state()
    if current in (FirmwareState.ESTOP, FirmwareState.ERROR):
        robot.reset_estop()
    robot.set_state(FirmwareState.RUNNING)


def reset_odometry(robot: Robot) -> None:
    robot.reset_odometry()
    if not robot.wait_for_odometry_reset(timeout=2.0):
        print("[warn] odometry reset not confirmed; continuing")
        robot.wait_for_pose_update(timeout=0.5)


# ===========================================================================
# ── Stage 1 helpers : traffic-light detection ──────────────────────────────
# ===========================================================================

def find_green_traffic_light(robot: Robot) -> bool:
    """Return True if a green traffic light is detected with sufficient confidence."""
    if not robot.is_vision_active(timeout_s=VISION_STALE_SEC):
        return False
    for detection in robot.get_detections("traffic light"):
        if float(detection["confidence"]) < MIN_TRAFFIC_LIGHT_CONFIDENCE:
            continue
        color = detection.get("attributes", {}).get("color", {}).get("value")
        if color == "green":
            return True
    return False


# ===========================================================================
# ── Stage 2 helpers : assembly (lift + gripper) ────────────────────────────
# ===========================================================================

def lift_move(robot: Robot, steps: int) -> bool:
    return robot.step_move(
        LIFT_STEPPER,
        steps=steps,
        move_type=StepMoveType.RELATIVE,
        blocking=True,
        timeout=LIFT_MOVE_TIMEOUT_S,
    )


def raise_platform(robot: Robot) -> bool:
    robot.step_set_config(
        LIFT_STEPPER,
        max_velocity=LIFT_MAX_VELOCITY,
        acceleration=LIFT_ACCELERATION,
    )
    robot.step_enable(LIFT_STEPPER)
    print("[MANIP] raise platform")
    ok = lift_move(robot, LIFT_UP_STEPS)
    robot.step_disable(LIFT_STEPPER)
    if not ok:
        print("[warn] lift failed to raise")
        return False
    return True


def perform_pick(robot: Robot) -> bool:
    """Open gripper → lower → close → raise.  Platform must already be raised."""
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


def pick_at_point(robot: Robot, label: str) -> bool:
    """Turn left 90° → nudge forward → pick → return → turn right 90°."""
    print(f"[MOTION] {label} — turn left 90°")
    robot.turn_by(delta_deg=90.0, blocking=True, tolerance_deg=TURN_TOLERANCE_DEG)

    print(f"[MOTION] {label} — move {PICK_SIDE_OFFSET_MM:.0f} mm toward item")
    robot.move_forward(
        distance=PICK_SIDE_OFFSET_MM,
        velocity=DRIVE_VELOCITY_MM_S,
        tolerance=DRIVE_TOLERANCE_MM,
        blocking=True,
    )

    if not perform_pick(robot):
        return False

    print(f"[MOTION] {label} — return {PICK_SIDE_OFFSET_MM:.0f} mm to line")
    robot.move_backward(
        distance=PICK_SIDE_OFFSET_MM,
        velocity=DRIVE_VELOCITY_MM_S,
        tolerance=DRIVE_TOLERANCE_MM,
        blocking=True,
    )

    print(f"[MOTION] {label} — turn right 90° (revert)")
    robot.turn_by(delta_deg=-90.0, blocking=True, tolerance_deg=TURN_TOLERANCE_DEG)

    print(f"[MOTION] {label} pick complete")
    return True


def run_assembly(robot: Robot) -> bool:
    """Full P2 → P1 → P3 pick sequence.  Returns False on any failure."""

    print("[ASSEMBLY] drive forward to P2")
    if not robot.move_forward(
        distance=DIST_START_TO_P2_MM,
        velocity=DRIVE_VELOCITY_MM_S,
        tolerance=DRIVE_TOLERANCE_MM,
        blocking=True,
    ):
        print("[warn] drive to P2 failed")
        robot.stop()
        return False

    if not pick_at_point(robot, "P2"):
        return False

    print("[ASSEMBLY] drive backward to P1")
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

    print("[ASSEMBLY] drive forward to P3")
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


# ===========================================================================
# ── Stage 3 helpers : navigation ───────────────────────────────────────────
# ===========================================================================

def print_nav_status(robot: Robot) -> None:
    ox, oy, otheta = robot.get_odometry_pose()
    if ENABLE_GPS and robot.has_fused_pose():
        fx, fy, ftheta = robot.get_fused_pose()
        print(
            f"  odom=({ox:6.0f}, {oy:6.0f}) mm  θ_odom={otheta:5.1f}°  |  "
            f"fused=({fx:6.0f}, {fy:6.0f}) mm  θ_fused={ftheta:5.1f}°  "
            f"gps={'fresh' if robot.is_gps_active() else 'stale'}"
        )
    else:
        print(f"  odom=({ox:6.0f}, {oy:6.0f}) mm  θ={otheta:5.1f}°")


# ===========================================================================
# ── Stage 4 helpers : person detection ─────────────────────────────────────
# ===========================================================================

def scan_for_person(robot: Robot) -> str | None:
    """Return the best-confidence customer_a/customer_b detection, or None."""
    detections = robot.get_detections()
    if not detections:
        return None
    best_class = None
    best_conf  = -1.0
    for d in detections:
        cls  = d.get("class_name", "")
        conf = float(d.get("confidence", 0.0))
        if cls in ("customer_a", "customer_b") and conf >= MIN_PERSON_CONFIDENCE:
            if conf > best_conf:
                best_conf  = conf
                best_class = cls
    return best_class


# ===========================================================================
# ── Stage 5 helpers : delivery ─────────────────────────────────────────────
# ===========================================================================

def open_gripper(robot: Robot) -> None:
    """Enable servo, open jaw, wait to settle, then disable."""
    robot.enable_servo(GRIPPER_SERVO)
    robot.set_servo(GRIPPER_SERVO, GRIPPER_OPEN_DEG)
    time.sleep(GRIPPER_SETTLE_S)
    robot.disable_servo(GRIPPER_SERVO)


def run_delivery(robot: Robot, customer: str) -> bool:
    """
    Execute the delivery sequence for the detected customer.

      1. Turn right 90°
      2. Drive forward DIST_CUSTOMER_A/B_MM
      3. Turn left 90°  (same direction as assembly pick)
      4. Open gripper
      5. Revert left turn (turn right 90°)
      6. Drive forward (DIST − RETURN_OFFSET_MM)
      7. Stop 2 s
      8. Drive forward FINAL_FORWARD_MM
    """
    if customer == "customer_a":
        dist_to_customer = DIST_CUSTOMER_A_MM
        label = "Customer A"
    else:
        dist_to_customer = DIST_CUSTOMER_B_MM
        label = "Customer B"

    return_dist = max(0.0, dist_to_customer - RETURN_OFFSET_MM)

    # 1. Turn right 90°
    print(f"[DELIVERY] {label} — turn right 90°")
    robot.turn_by(delta_deg=-90.0, blocking=True, tolerance_deg=TURN_TOLERANCE_DEG)

    # 2. Drive to customer
    print(f"[DELIVERY] {label} — drive {dist_to_customer:.0f} mm to customer")
    if not robot.move_forward(
        distance=dist_to_customer,
        velocity=DELIVERY_VELOCITY_MM_S,
        tolerance=DELIVERY_TOLERANCE_MM,
        blocking=True,
    ):
        print("[warn] drive to customer failed")
        robot.stop()
        return False

    # 3. Turn left 90°  (mirrors assembly pick turn)
    print(f"[DELIVERY] {label} — turn left 90°")
    robot.turn_by(delta_deg=90.0, blocking=True, tolerance_deg=TURN_TOLERANCE_DEG)

    # 4. Open gripper to release item
    print(f"[DELIVERY] {label} — open gripper (release)")
    open_gripper(robot)

    # 5. Revert left turn (turn right 90°) → face along driving axis again
    print(f"[DELIVERY] {label} — turn right 90° (revert)")
    robot.turn_by(delta_deg=-90.0, blocking=True, tolerance_deg=TURN_TOLERANCE_DEG)

    # 6. Drive back along the delivery axis by (dist − offset)
    print(f"[DELIVERY] {label} — drive {return_dist:.0f} mm forward (return leg)")
    if not robot.move_forward(
        distance=return_dist,
        velocity=DELIVERY_VELOCITY_MM_S,
        tolerance=DELIVERY_TOLERANCE_MM,
        blocking=True,
    ):
        print("[warn] return drive failed")
        robot.stop()
        return False

    # 7. Pause 2 seconds
    print(f"[DELIVERY] {label} — pausing 2 s")
    time.sleep(2.0)

    # 8. Final forward push
    print(f"[DELIVERY] {label} — final drive {FINAL_FORWARD_MM:.0f} mm")
    if not robot.move_forward(
        distance=FINAL_FORWARD_MM,
        velocity=DELIVERY_VELOCITY_MM_S,
        tolerance=DELIVERY_TOLERANCE_MM,
        blocking=True,
    ):
        print("[warn] final drive failed")
        robot.stop()
        return False

    robot.stop()
    print(f"[DELIVERY] {label} — complete")
    return True


# ===========================================================================
# ── Main FSM ────────────────────────────────────────────────────────────────
# ===========================================================================

def run(robot: Robot) -> None:
    configure_robot(robot)

    state = "INIT"

    # Stage 3 (nav) bookkeeping
    drive_handle      = None
    last_status_at    = 0.0

    # Stage 4 (person detect) bookkeeping
    confirm_class     = None   # class currently being confirmed
    confirm_start     = 0.0    # when the current confirmation window started
    detected_customer = None   # result locked in after hold period

    period    = 1.0 / float(DEFAULT_FSM_HZ)
    next_tick = time.monotonic()

    while True:
        now = time.monotonic()

        # ── INIT ─────────────────────────────────────────────────────────────
        if state == "INIT":
            start_robot(robot)
            reset_odometry(robot)
            dim_all_leds(robot)
            show_idle_leds(robot)
            print("[FSM] IDLE — press BTN_1 to begin the full mission")
            print(f"[CFG] scan angle α = {TRAFFIC_LIGHT_SCAN_ALPHA_DEG:.1f}°")
            print(f"[CFG] pick order: P2 → P1 → P3")
            print(f"[CFG] nav waypoints: {len(PATH_CONTROL_POINTS)}")
            print(f"[CFG] delivery: A={DIST_CUSTOMER_A_MM:.0f} mm  B={DIST_CUSTOMER_B_MM:.0f} mm")
            state = "IDLE"

        # ── IDLE — wait for BTN_1 ────────────────────────────────────────────
        elif state == "IDLE":
            if robot.was_button_pressed(Button.BTN_1):
                show_running_leds(robot)
                reset_odometry(robot)
                print("[FSM] ── STAGE 1 : Traffic-light scan ──")
                print(f"[MOTION] turn to scan angle {TRAFFIC_LIGHT_SCAN_ALPHA_DEG:.1f}°")
                robot.turn_by(
                    delta_deg=TRAFFIC_LIGHT_SCAN_ALPHA_DEG,
                    blocking=True,
                    tolerance_deg=TRAFFIC_LIGHT_TURN_TOLERANCE_DEG,
                )
                state = "SCAN_TRAFFIC"

        # ── STAGE 1 : scan for green light ───────────────────────────────────
        elif state == "SCAN_TRAFFIC":
            # Spins here every FSM tick until green is detected
            if find_green_traffic_light(robot):
                print("[VISION] green light detected — returning to neutral")
                robot.turn_by(
                    delta_deg=-TRAFFIC_LIGHT_SCAN_ALPHA_DEG,
                    blocking=True,
                    tolerance_deg=TRAFFIC_LIGHT_TURN_TOLERANCE_DEG,
                )
                reset_odometry(robot)
                print("[FSM] ── STAGE 2 : Assembly ──")
                state = "ASSEMBLY"

        # ── STAGE 2 : assembly pick sequence ─────────────────────────────────
        elif state == "ASSEMBLY":
            ok = run_assembly(robot)
            show_idle_leds(robot)
            if not ok:
                print("[FSM] assembly failed — returning to INIT")
                state = "INIT"
            else:
                print("[FSM] assembly complete")
                reset_odometry(robot)
                print("[FSM] ── STAGE 3 : Navigation ──")
                show_running_leds(robot)
                print(f"[NAV] starting blended path ({len(PATH_CONTROL_POINTS)} waypoints)")
                drive_handle   = robot.vector_blended_follow_path(
                    waypoints=PATH_CONTROL_POINTS,
                    velocity=NAV_VELOCITY_MM_S,
                    lookahead=NAV_LOOKAHEAD_MM,
                    tolerance=NAV_TOLERANCE_MM,
                    repulsion_range=NAV_REPULSION_RANGE_MM,
                    repulsion_gain=NAV_REPULSION_GAIN,
                    advance_radius=NAV_ADVANCE_RADIUS_MM,
                    max_angular_rad_s=NAV_MAX_ANGULAR_RAD_S,
                    blocking=False,
                )
                last_status_at = now
                state = "NAVIGATING"

        # ── STAGE 3 : navigation ─────────────────────────────────────────────
        elif state == "NAVIGATING":
            if now - last_status_at >= NAV_STATUS_INTERVAL_S:
                print_nav_status(robot)
                last_status_at = now

            if drive_handle is not None and drive_handle.is_finished():
                print("[NAV] path complete")
                print_nav_status(robot)
                drive_handle = None
                robot.stop()
                # reset confirm state before entering detection
                confirm_class     = None
                confirm_start     = 0.0
                detected_customer = None
                print("[FSM] ── STAGE 4 : Person detection ──")
                print("[VISION] scanning for customer_a or customer_b ...")
                state = "DETECT_PERSON"

        # ── STAGE 4 : person detection with hold confirmation ─────────────────
        elif state == "DETECT_PERSON":
            person = scan_for_person(robot)

            if person is None:
                # No detection this tick — reset the confirmation window
                if confirm_class is not None:
                    print("[VISION] detection lost — restarting confirmation")
                confirm_class = None
                confirm_start = 0.0

            elif person != confirm_class:
                # New (or different) class seen — start a fresh window
                print(f"[VISION] candidate: {person} — starting confirmation hold")
                confirm_class = person
                confirm_start = now

            else:
                # Same class seen again — check if hold period has elapsed
                elapsed = now - confirm_start
                if elapsed >= PERSON_CONFIRM_HOLD_S:
                    detected_customer = confirm_class
                    print(f"[VISION] confirmed: {detected_customer} "
                          f"(held {elapsed:.2f} s ≥ {PERSON_CONFIRM_HOLD_S:.2f} s)")
                    show_running_leds(robot)
                    print("[FSM] ── STAGE 5 : Delivery ──")
                    state = "DELIVER"

        # ── STAGE 5 : delivery ───────────────────────────────────────────────
        elif state == "DELIVER":
            ok = run_delivery(robot, detected_customer)
            show_idle_leds(robot)
            if ok:
                print("[FSM] ── MISSION COMPLETE — press BTN_1 to run again ──")
                state = "IDLE"
            else:
                print("[FSM] delivery failed — returning to INIT")
                state = "INIT"

        # ── tick-rate control ────────────────────────────────────────────────
        next_tick += period
        sleep_s = next_tick - now
        if sleep_s > 0.0:
            time.sleep(sleep_s)
        else:
            next_tick = time.monotonic()