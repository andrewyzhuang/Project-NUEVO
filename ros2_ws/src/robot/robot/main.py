from __future__ import annotations
import time

from robot.robot import FirmwareState, Robot, Unit
from robot.hardware_map import Button, DEFAULT_FSM_HZ, LED, Motor,POSITION_UNIT
from robot.util import densify_polyline
from robot.path_planner import PurePursuitPlanner
import math
import numpy as np

# ---------------------------------------------------------------------------
# Robot build configuration
# ---------------------------------------------------------------------------

LED_BRIGHTNESS = 255
LIGHT_HOLD_SEC = 2.0
VISION_STALE_SEC = 3.0
MIN_TRAFFIC_LIGHT_CONFIDENCE = 0.50

TAG_ID = 21 # set aruco tag ID 
POSITION_UNIT = Unit.MM
WHEEL_DIAMETER = 74.0
WHEEL_BASE = 333.0
INITIAL_THETA_DEG = 90.0

LEFT_WHEEL_MOTOR = Motor.DC_M1
LEFT_WHEEL_DIR_INVERTED = False
RIGHT_WHEEL_MOTOR = Motor.DC_M2
RIGHT_WHEEL_DIR_INVERTED = True


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
    robot.set_tracked_tag_id(TAG_ID) # set aruco tag ID as the tracked tag for localization
    robot.enable_vision()


def show_idle_leds(robot: Robot) -> None:
    robot.set_led(LED.GREEN, 0)
    robot.set_led(LED.ORANGE, 255)


def show_moving_leds(robot: Robot) -> None:
    robot.set_led(LED.ORANGE, 0)
    robot.set_led(LED.GREEN, 255)


def start_robot(robot: Robot) -> None:
    robot.set_state(FirmwareState.RUNNING)
    robot.reset_odometry()
    robot.wait_for_pose_update(timeout=0.2)


def find_traffic_light_color(robot: Robot) -> str | None:
    """Return the best recent red/green traffic-light result, or None."""
    if not robot.is_vision_active(timeout_s=VISION_STALE_SEC):
        return None

    best_color = None
    best_confidence = -1.0

    for detection in robot.get_detections("traffic light"):
        confidence = float(detection["confidence"])
        if confidence < MIN_TRAFFIC_LIGHT_CONFIDENCE:
            continue

        attributes = detection.get("attributes", {})
        color_attribute = attributes.get("color", {})
        color = color_attribute.get("value")
        if color not in ("red", "green"):
            continue

        if confidence > best_confidence:
            best_confidence = confidence
            best_color = str(color)

    return best_color


def show_traffic_light_color(robot: Robot, color: str) -> None:
    if color == "red":
        robot.set_led(LED.RED, LED_BRIGHTNESS)
        robot.set_led(LED.GREEN, 0)
    elif color == "green":
        robot.set_led(LED.RED, 0)
        robot.set_led(LED.GREEN, LED_BRIGHTNESS)


def run(robot: Robot) -> None:
    configure_robot(robot)
    state = "INIT"
    drive_handle = None
    lights_off_at = 0.0
    last_shown_color = None

    period = 1.0 / float(DEFAULT_FSM_HZ)
    print(f"FSM period: {period:.3f} seconds")
    next_tick = time.monotonic()

    while True:
        if state == "INIT":
            start_robot(robot)
            print("[FSM] INIT (odometry reset)")
            # center lane
            # path_control_points = [
            #     (0.0,   0.0),
            #     (0.0, 2500.0),
            #     (1000.0, 2500.0),
            # ]
            # left lane
            path_control_points = [
                (300.0,   0.0),
                (300.0, 2500.0),
                (1300.0, 2500.0),
            ]

            path = densify_polyline(path_control_points, spacing=400.0)

            robot._nav_follow_pp_path(
                lookahead_distance=100.0,
                max_linear_speed=140.0,
                max_angular_speed=1.5,
                goal_tolerance=20.0,
                obstacles_range=450.0,
                view_angle=math.radians(70.0),
                safe_dist=250.0,
                avoidance_delay=150,
                alpha_Ld=0.7,
                offset=270.0,
                lane_width=500.0,
                obstacle_avoidance=True,
                x_L=300.0,
            )
            robot.planner.set_path(path)
            print("Path is ready, Entering IDLE state.")
            print("[FSM] IDLE - Press BTN_1 to enter MOVING state.")
            state = "IDLE"

        elif state == "IDLE":
            show_idle_leds(robot)
            robot._draw_lidar_obstacles()
            if robot.get_button(Button.BTN_1):
                print("Start Scanning Traffic Light")
                state = "SCANNING TRAFFIC LIGHT"
            if robot.get_button(Button.BTN_2):
                print("BTN_2 pressed. Stopping robot and saving trajectory.")
                robot.shutdown()
        
        elif state == "SCANNING TRAFFIC LIGHT":
            now = time.monotonic()
            traffic_light_color = find_traffic_light_color(robot)

            if traffic_light_color in ("red", "green"):
                show_traffic_light_color(robot, traffic_light_color)
                lights_off_at = now + LIGHT_HOLD_SEC

                if traffic_light_color != last_shown_color:
                    print(f"[VISION] traffic light: {traffic_light_color}")
                last_shown_color = traffic_light_color
                
                if traffic_light_color == "green":
                    state = "MOVING"

            elif lights_off_at > 0.0 and now >= lights_off_at:
                robot.set_led(LED.RED, 0)
                robot.set_led(LED.GREEN, 0)
                lights_off_at = 0.0
                if last_shown_color is not None:
                    print("[VISION] no recent red/green light - LEDs off")
                last_shown_color = None

        elif state == "MOVING":
            show_moving_leds(robot)
            # if next_tick % 0.5 < period: # print every half second
            #     robot._draw_lidar_obstacles()
            #     print("Obstacle figure updated.")
            state = robot._nav_follow_pp_path_loop()

        # FSM refresh rate control
        next_tick += period
        sleep_s = next_tick - time.monotonic()
        if sleep_s > 0.0:
            time.sleep(sleep_s)
        else:
            next_tick = time.monotonic()