"""
main.py - Integrated FSM for Burger Assembly, Navigation, and Drop-off
======================================================================
This script integrates vision-based traffic light detection, robotic arm 
manipulation for burger assembly, vector-blended pure-pursuit navigation, 
and vision-based person detection for dynamic drop-off routing.

HOW TO RUN
----------
    cp main.py ros2_ws/src/robot/robot/main.py
    ros2 run robot robot

Press BTN_1 to start the full sequence. BTN_2 cancels and returns to IDLE.
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
    LIDAR_FOV_DEG,
    LIDAR_MOUNT_THETA_DEG,
    LIDAR_MOUNT_X_MM,
    LIDAR_MOUNT_Y_MM,
    LIDAR_RANGE_MAX_MM,
    LIDAR_RANGE_MIN_MM,
)
from robot.robot import FirmwareState, Robot

# ---------------------------------------------------------------------------
# Configuration & Tunable Constants
# ---------------------------------------------------------------------------

# --- Vision ---
MIN_TRAFFIC_LIGHT_CONFIDENCE = 0.50
VISION_STALE_SEC = 3.0

# --- Gripper (Servo 1) ---
GRIPPER_SERVO       = ServoChannel.CH_1
GRIPPER_OPEN_DEG    = 135.0
GRIPPER_CLOSE_DEG   = 167.0
GRIPPER_SETTLE_S    = 1.0

# --- Lift (Stepper 1) ---
LIFT_STEPPER        = Stepper.STEPPER_1
LIFT_MAX_VELOCITY   = 800
LIFT_ACCELERATION   = 400
LIFT_MOVE_TIMEOUT_S = 10.0

HEIGHT_1_STEPS = 0       # Top of lift travel
HEIGHT_2_STEPS = 2475    # Table/platform surface level
HEIGHT_3_STEPS = 1875    # Patty height
HEIGHT_4_STEPS = 1875    # Bun height
HEIGHT_5_STEPS = 1700    # Clearance height

# --- Drive & Geometry ---
DRIVE_VELOCITY_MM_S  = 90.0
DRIVE_TOLERANCE_MM   = 20.0
TURN_TOLERANCE_DEG   = 2.0

DIST_START_TO_P1_MM  = 1125.0
DIST_P1_TO_P2_MM     = 150.0
DIST_P2_TO_P3_MM     = 150.0
PICK_SIDE_OFFSET_MM  = 40.0

# --- Path Planning (Vector Blended) ---
ENABLE_LIDAR = True
PATH_CONTROL_POINTS = [
    (0.0, 0.0, 240.0),
    (0.0, 2150.0, 240.0),
    (520.0, 2150.0, 160.0),
    (520.0, -700.0, 240.0),
    (1300.0, -700.0, 240.0),
    (1300.0, 1950.0, 360.0),
    (1300.0, 2100.0, 160.0)
]
VELOCITY_MM_S      = 150.0
LOOKAHEAD_MM       = 150.0
TOLERANCE_MM       = 25.0
ADVANCE_RADIUS_MM  = 100.0
MAX_ANGULAR_RAD_S  = 0.6
REPULSION_RANGE_MM = 450.0
REPULSION_GAIN     = 600.0

STATUS_PRINT_INTERVAL_S = 1.0


# ---------------------------------------------------------------------------
# Core Helpers
# ---------------------------------------------------------------------------

class CancelFlag:
    def __init__(self) -> None:
        self.cancelled = False
    def cancel(self) -> None:
        self.cancelled = True
    def is_set(self) -> bool:
        return self.cancelled


def configure_robot(robot: Robot) -> None:
    robot.set_unit(POSITION_UNIT)
    robot.enable_vision()
    robot.set_odometry_parameters(
        wheel_diameter=WHEEL_DIAMETER,
        wheel_base=WHEEL_BASE,
        initial_theta_deg=INITIAL_THETA_DEG,
        left_motor_id=LEFT_WHEEL_MOTOR,
        right_motor_id=RIGHT_WHEEL_MOTOR,
        left_motor_dir_inverted=LEFT_WHEEL_DIR_INVERTED,
        right_motor_dir_inverted=RIGHT_WHEEL_DIR_INVERTED,
    )
    if ENABLE_LIDAR:
        robot.enable_lidar()
        robot.set_lidar_mount(x_mm=LIDAR_MOUNT_X_MM, y_mm=LIDAR_MOUNT_Y_MM, theta_deg=LIDAR_MOUNT_THETA_DEG)
        robot.set_lidar_filter(range_min_mm=LIDAR_RANGE_MIN_MM, range_max_mm=LIDAR_RANGE_MAX_MM, fov_deg=LIDAR_FOV_DEG)
        robot.start_lidar_world_publisher()

def start_robot(robot: Robot) -> None:
    if robot.get_state() in (FirmwareState.ESTOP, FirmwareState.ERROR):
        robot.reset_estop()
    robot.set_state(FirmwareState.RUNNING)

def show_idle_leds(robot: Robot) -> None:
    robot.set_led(LED.ORANGE, 200)
    robot.set_led(LED.GREEN, 0)
    robot.set_led(LED.BLUE, 0)

def print_status(robot: Robot) -> None:
    ox, oy, otheta = robot.get_odometry_pose()
    print(f"  odom=({ox:6.0f}, {oy:6.0f}) mm  θ={otheta:5.1f}°")

# --- Vision Helpers ---
def find_traffic_light_color(robot: Robot) -> str | None:
    if not robot.is_vision_active(timeout_s=VISION_STALE_SEC): return None
    best_color, best_confidence = None, -1.0
    for detection in robot.get_detections("traffic light"):
        conf = float(detection["confidence"])
        if conf < MIN_TRAFFIC_LIGHT_CONFIDENCE: continue
        color = detection.get("attributes", {}).get("color", {}).get("value")
        if color in ("red", "green") and conf > best_confidence:
            best_confidence, best_color = conf, str(color)
    return best_color

def scan_for_person(robot: Robot) -> str | None:
    if not robot.is_vision_active(timeout_s=VISION_STALE_SEC): return None
    for d in robot.get_detections():
        if d["class_name"] in ("person_1", "person_2"):
            return d["class_name"]
    return None

# --- Manipulator Helpers ---
def lift_enable(robot: Robot) -> None:
    robot.step_set_config(LIFT_STEPPER, max_velocity=LIFT_MAX_VELOCITY, acceleration=LIFT_ACCELERATION)
    robot.step_enable(LIFT_STEPPER)

def lift_to_height(robot: Robot, from_steps: int, to_steps: int) -> bool:
    delta = to_steps - from_steps
    if delta == 0: return True
    return robot.step_move(LIFT_STEPPER, steps=-delta, move_type=StepMoveType.RELATIVE, blocking=True, timeout=LIFT_MOVE_TIMEOUT_S)

def execute_pick(robot: Robot, cancel: CancelFlag, current_height: int, target_height: int, label: str) -> tuple[bool, int]:
    """Generic pick sequence to reduce redundant code."""
    if cancel.is_set(): return False, current_height
    robot.turn_by(delta_deg=90.0, blocking=True, tolerance_deg=TURN_TOLERANCE_DEG)
    robot.move_forward(distance=PICK_SIDE_OFFSET_MM, velocity=DRIVE_VELOCITY_MM_S, tolerance=DRIVE_TOLERANCE_MM, blocking=True)
    
    lift_enable(robot)
    if target_height != HEIGHT_5_STEPS: # If we need to go down to patty/bun height first
        lift_to_height(robot, current_height, target_height)
        current_height = target_height

    robot.enable_servo(GRIPPER_SERVO)
    robot.set_servo(GRIPPER_SERVO, GRIPPER_OPEN_DEG)
    time.sleep(GRIPPER_SETTLE_S)

    lift_to_height(robot, current_height, HEIGHT_2_STEPS)
    current_height = HEIGHT_2_STEPS
    
    robot.set_servo(GRIPPER_SERVO, GRIPPER_CLOSE_DEG)
    time.sleep(GRIPPER_SETTLE_S)

    lift_to_height(robot, current_height, HEIGHT_5_STEPS)
    current_height = HEIGHT_5_STEPS
    robot.step_disable(LIFT_STEPPER)

    robot.move_backward(distance=PICK_SIDE_OFFSET_MM, velocity=DRIVE_VELOCITY_MM_S, tolerance=DRIVE_TOLERANCE_MM, blocking=True)
    robot.turn_by(delta_deg=-90.0, blocking=True, tolerance_deg=TURN_TOLERANCE_DEG)
    return True, current_height

def run_assembly_sequence(robot: Robot, cancel: CancelFlag) -> bool:
    current_height = HEIGHT_2_STEPS
    
    # P1
    if not robot.move_forward(DIST_START_TO_P1_MM, DRIVE_VELOCITY_MM_S, DRIVE_TOLERANCE_MM, blocking=True): return False
    lift_enable(robot)
    lift_to_height(robot, current_height, HEIGHT_5_STEPS)
    current_height = HEIGHT_5_STEPS
    robot.step_disable(LIFT_STEPPER)
    ok, current_height = execute_pick(robot, cancel, current_height, HEIGHT_5_STEPS, "P1")
    if not ok: return False

    # P2
    if not robot.move_forward(DIST_P1_TO_P2_MM, DRIVE_VELOCITY_MM_S, DRIVE_TOLERANCE_MM, blocking=True): return False
    ok, current_height = execute_pick(robot, cancel, current_height, HEIGHT_3_STEPS, "P2")
    if not ok: return False

    # P3
    if not robot.move_forward(DIST_P2_TO_P3_MM, DRIVE_VELOCITY_MM_S, DRIVE_TOLERANCE_MM, blocking=True): return False
    ok, current_height = execute_pick(robot, cancel, current_height, HEIGHT_4_STEPS, "P3")
    if not ok: return False

    # Return to HEIGHT_3
    lift_enable(robot)
    lift_to_height(robot, current_height, HEIGHT_3_STEPS)
    robot.step_disable(LIFT_STEPPER)
    return True


# ---------------------------------------------------------------------------
# Main Routine & FSM
# ---------------------------------------------------------------------------

def run(robot: Robot) -> None:
    configure_robot(robot)

    state = "INIT"
    cancel = CancelFlag()
    drive_handle = None
    last_status_print_at = 0.0
    detected_person = None

    period = 1.0 / float(DEFAULT_FSM_HZ)
    next_tick = time.monotonic()

    while True:
        now = time.monotonic()
        # Global BTN_2 Cancel
        if robot.was_button_pressed(Button.BTN_2):
            if drive_handle is not None:
                drive_handle.cancel()
                drive_handle.wait(timeout=1.0)
            robot.stop()
            robot.step_disable(LIFT_STEPPER)
            robot.disable_servo(GRIPPER_SERVO)
            show_idle_leds(robot)
            cancel.cancel()
            print("[FSM] TERMINATED by BTN_2")
            state = "IDLE"

        # -------------------------------------------------------------------
        # FSM STATES
        # -------------------------------------------------------------------
        if state == "INIT":
            start_robot(robot)
            robot.reset_odometry()
            show_idle_leds(robot)
            print("[FSM] READY - Press BTN_1 to commence buger building")
            state = "IDLE"

        elif state == "IDLE":
            if robot.was_button_pressed(Button.BTN_1):
                cancel = CancelFlag()
                robot.set_led(LED.GREEN, 255)
                
                # Lower platform from HEIGHT_1 → HEIGHT_5 before scanning
                lift_enable(robot)
                print("[MANIP] lower platform to HEIGHT_2 for scanning")
                if not lift_to_height(robot, HEIGHT_1_STEPS, HEIGHT_2_STEPS):
                    print("[warn] lift failed to reach HEIGHT_2 — returning to INIT")
                    robot.step_disable(LIFT_STEPPER)
                    show_idle_leds(robot)
                    state = "INIT"
                else:
                    robot.step_disable(LIFT_STEPPER)
                    print("[MOTION] Turning 20 deg left to face traffic light...")
                    robot.turn_by(20.0, blocking=True, tolerance_deg=TURN_TOLERANCE_DEG)
                    print("[VISION] Waiting for GREEN light...")
                    state = "AWAIT_TRAFFIC_LIGHT"

        elif state == "AWAIT_TRAFFIC_LIGHT":
            color = find_traffic_light_color(robot)
            if color == "green":
                print("[VISION] Green light detected! Turning back...")
                robot.turn_by(-20.0, blocking=True, tolerance_deg=TURN_TOLERANCE_DEG)
                state = "ASSEMBLY"

        elif state == "ASSEMBLY":
            print("[MANIP] Starting Buger Assembly...")
            success = run_assembly_sequence(robot, cancel)
            if success and not cancel.is_set():
                state = "START_NAVIGATION"
            else:
                state = "IDLE"

        elif state == "START_NAVIGATION":
            print("[MOTION] Assembly complete. Resetting Odometry for Navigation.")
            #robot.turn_by(delta_deg=6.0, blocking=True, tolerance_deg=TURN_TOLERANCE_DEG)
            robot.reset_odometry()
            if not robot.wait_for_odometry_reset(timeout=2.0):
                robot.wait_for_pose_update(timeout=0.5)
            
            drive_handle = robot.vector_blended_follow_path(
                waypoints=PATH_CONTROL_POINTS,
                velocity=VELOCITY_MM_S,
                lookahead=LOOKAHEAD_MM,
                tolerance=TOLERANCE_MM,
                repulsion_range=REPULSION_RANGE_MM,
                repulsion_gain=REPULSION_GAIN,
                advance_radius=ADVANCE_RADIUS_MM,
                max_angular_rad_s=MAX_ANGULAR_RAD_S,
                blocking=False,
            )
            state = "NAVIGATING"

        elif state == "NAVIGATING":
            if now - last_status_print_at >= STATUS_PRINT_INTERVAL_S:
                    print_status(robot)
                    last_status_print_at = now
            if drive_handle is not None and drive_handle.is_finished():
                print("[MOTION] Path complete. Searching for person...")
                drive_handle = None
                robot.turn_to(0.0, blocking=True, tolerance_deg=TURN_TOLERANCE_DEG)
                robot.stop()
                state = "DETECT_PERSON"

        elif state == "DETECT_PERSON":
            person = scan_for_person(robot)
            if person in ("person_1", "person_2"):
                detected_person = person
                print(f"[VISION] Detected {detected_person}. Executing final approach.")
                robot.move_forward(720.0, velocity=DRIVE_VELOCITY_MM_S, tolerance=DRIVE_TOLERANCE_MM, blocking=True)
                # 5) Turn right (-90 deg), move specific distance
                robot.turn_to(-90.0, blocking=True, tolerance_deg=TURN_TOLERANCE_DEG)

                state = "START_DELIVERY"

        elif state == "START_DELIVERY":
            print("[MOTION] Resetting Odometry for Delivery.")
            robot.reset_odometry()
            if not robot.wait_for_odometry_reset(timeout=2.0):
                robot.wait_for_pose_update(timeout=0.5)
            
            dist_to_platform = 2300.0 if detected_person == "person_1" else 2000.0

            delivery_waypoints = [
                (0.0, 0.0, 240.0),
                (0.0, dist_to_platform, 240.0)
            ]

            drive_handle = robot.vector_blended_follow_path(
                waypoints=delivery_waypoints,
                velocity=VELOCITY_MM_S,
                lookahead=LOOKAHEAD_MM,
                tolerance=TOLERANCE_MM,
                repulsion_range=REPULSION_RANGE_MM,
                repulsion_gain=REPULSION_GAIN,
                advance_radius=ADVANCE_RADIUS_MM,
                max_angular_rad_s=MAX_ANGULAR_RAD_S,
                blocking=False,
            )
            state = "DELIVERY"

        elif state == "DELIVERY":
            if now - last_status_print_at >= STATUS_PRINT_INTERVAL_S:
                    print_status(robot)
                    last_status_print_at = now
            if drive_handle is not None and drive_handle.is_finished():
                print("[MOTION] Path complete. Dropping off...")
                drive_handle = None
                robot.stop()
                state = "DROPOFF"


        elif state == "DROPOFF":
            # Turn left (90 deg) to face platform, drop off
            robot.turn_by(90.0, blocking=True, tolerance_deg=TURN_TOLERANCE_DEG)
            #robot.move_forward(distance=PICK_SIDE_OFFSET_MM, velocity=DRIVE_VELOCITY_MM_S, tolerance=DRIVE_TOLERANCE_MM, blocking=True)

            print("[MANIP] Dropping off buger...")
            lift_enable(robot)
            lift_to_height(robot, HEIGHT_3_STEPS, HEIGHT_2_STEPS)  # Lower to table
            robot.enable_servo(GRIPPER_SERVO)
            robot.set_servo(GRIPPER_SERVO, GRIPPER_OPEN_DEG)       # Open gripper
            time.sleep(GRIPPER_SETTLE_S)
            lift_to_height(robot, HEIGHT_2_STEPS, HEIGHT_1_STEPS)  # Raise to top
            robot.step_disable(LIFT_STEPPER)

            #robot.move_backward(distance=PICK_SIDE_OFFSET_MM, velocity=DRIVE_VELOCITY_MM_S, tolerance=DRIVE_TOLERANCE_MM, blocking=True)

            # Turn right (-90 deg) to face course, move, wait, move
            robot.turn_by(-90.0, blocking=True, tolerance_deg=TURN_TOLERANCE_DEG)
            dist_final = 400.0 if detected_person == "person_1" else 700.0
            robot.move_forward(dist_final, velocity=DRIVE_VELOCITY_MM_S, tolerance=DRIVE_TOLERANCE_MM, blocking=True)
            
            print("[MOTION] Stopping for 2 seconds...")
            time.sleep(2.0) # Blocking sleep is acceptable here as it's the very end of the sequence
            
            robot.move_forward(600.0, velocity=DRIVE_VELOCITY_MM_S, tolerance=DRIVE_TOLERANCE_MM, blocking=True)
            print("[FSM] MISSION COMPLETE. Returning to IDLE.")
            
            show_idle_leds(robot)
            state = "IDLE"

        # -- Tick-rate control --
        next_tick += period
        sleep_s = next_tick - time.monotonic()
        if sleep_s > 0.0:
            time.sleep(sleep_s)
        else:
            next_tick = time.monotonic()