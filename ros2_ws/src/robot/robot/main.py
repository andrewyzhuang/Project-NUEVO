from __future__ import annotations
import time

from robot.robot import FirmwareState, Robot, Unit
from robot.hardware_map import Button, DEFAULT_FSM_HZ, LED, Motor
from robot.util import densify_polyline
from robot.path_planner import PurePursuitPlanner
import math
import numpy as np


# ---------------------------------------------------------------------------
# Robot build configuration
# ---------------------------------------------------------------------------

TAG_ID = 21 # set aruco tag ID 
TAG_ID = 21 # set aruco tag ID 
POSITION_UNIT = Unit.MM
WHEEL_DIAMETER = 74.0
WHEEL_BASE = 333.0
INITIAL_THETA_DEG = 90.0

LEFT_WHEEL_MOTOR = Motor.DC_M1
LEFT_WHEEL_DIR_INVERTED = False
RIGHT_WHEEL_MOTOR = Motor.DC_M2
RIGHT_WHEEL_DIR_INVERTED = True

#Venue and ingredient information
tile = 25.4 #lengths of tiles in lab (mm)
shelf_height = 152.4 #mm, need to verify actual height (6-8in)
bun_height = 20 #mm
patty_height = 15#mm
#important heights
target_heights = [shelf_height, shelf_height + bun_height,
            shelf_height, shelf_height + bun_height, patty_height, shelf_height]
assem_stage = 0 #initialize counter for assembly state
bottom_bun = 0 # used as a flag

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


def run(robot: Robot) -> None:
    configure_robot(robot)

    state = "INIT"
    drive_handle = None
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
                print("Start Scanning for Traffic Light!")
                state = "TRAFFIC SCAN"
            if robot.get_button(Button.BTN_2):
                print("BTN_2 pressed. Stopping robot and saving trajectory.")
                robot.shutdown()

        elif state == "TRAFFIC SCAN":
            #scan for green light 
            #if traffic light detected:
                print("Green light detected. Entering MOVE TO SHELVES state")
                state = "MOVE TO SHELVES"

        elif state == "MOVE TO SHELVES":
            show_moving_leds(robot)
            # if next_tick % 0.5 < period: # print every half second
            #     robot._draw_lidar_obstacles()
            #     print("Obstacle figure updated.")
            state = robot._nav_follow_pp_path_loop()
            #if distance traveled = 2 tiles (reached shelves):
            print("Entering ingredient SEARCH state")
            #   state == SEARCH
        
        elif state == "SEARCH":
            #error check for collision

            #scan for obstacles with lidar
            #adjust heading
            #drive foward 8 inches
            #scan table with camera
            #if bun detected:
                #if assem_stage == 0 and bottom_bun == 0:
                    #record bottom bun location, search for patty
                    print('Saving Bottom Bun Location. Continuing Search')
                    bottom_bun == 1
                #else:
                    print('Bun Identified. Entering Alignment State')
                    state = "ALIGNMENT"
                    
            #elif patty detected:
                #print('Patty identified. Entering ALIGNMENT state.')
                #state == "ALIGNMENT"

        elif state == "ALIGNMENT":
            #adjust heading towards ingredient using camera
            #drive forward
            #if table reached (Lidar <3 in):
                #print("Aligned with ingredient, Entering ASSEMBLY")
                #state = "ASSEMBLY"

        #elif state == "ASSEMBLY":
            ### MOVE LINEAR ACTUATOR TO CORRECT HEIGHT ###
           # target_height = target_heights[assem_stage]
            #get current height
            #move stepper to target_height - current height

            ## OPEN AND CLOSE GRIPPER ##
            if assem_stage % 2 == 0: #closing gripper(stages 0, 2, 4)
                #close gripper
            #else: #opening gripper (stages 1, 3)
                #open gripper
            #error check (nothing picked up)

            #assem_stage += 1 #update assembly stage counter
            
            #if assem_stage > 4: #fully assembled, move on to next state
                state = "DELIVERY"
            else:
                 state = "SEARCH"


        elif state == "DELIVERY":
            #navigate to delivery area (avoiding obstacles)
            #Turn 90 degrees and drive to drop off location
            #move linear actuator to correct height and open gripper

            #error check
            


        #FSM refresh rate control
        next_tick += period
        sleep_s = next_tick - time.monotonic()
        if sleep_s > 0.0:
            time.sleep(sleep_s)
        else:
            next_tick = time.monotonic()