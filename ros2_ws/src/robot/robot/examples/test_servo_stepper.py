import time
from robot.hardware_map import ServoChannel, Stepper, StepMoveType, POSITION_UNIT
from robot.robot import Robot, FirmwareState

# ===========================================================================
# USER CONFIGURATION: Set your desired angles and distances here
# ===========================================================================
# Servo Angles (in degrees)
GRIPPER_OPEN_DEG = 15.0     # Adjust this for your "open" state
GRIPPER_CLOSE_DEG = 120.0   # Adjust this for your "closed" state
GRIPPER_SETTLE_S = 1.5      # Time (seconds) to wait for the servo to finish moving

# Stepper Distance (in steps)
LIFT_STEPS = 1000     # Relative distance to extend forward
STEPPER_MAX_VELOCITY = 800      # Speed cap for the stepper
STEPPER_ACCELERATION = 400      # Acceleration ramp for the stepper
STEPPER_HOME_VELOCITY = 300     # Homing speed
# ===========================================================================

# Hardware Pins/Channels
GRIPPER_SERVO = ServoChannel.CH_1
STEPPER_LIFT = Stepper.STEPPER_1

def main():
    robot = Robot()
    robot.set_unit(POSITION_UNIT)
    
    # Clear any errors and enable the robot
    if robot.get_state() in (FirmwareState.ESTOP, FirmwareState.ERROR):
        robot.reset_estop()
    robot.set_state(FirmwareState.RUNNING)

    # 1. STEPPER SETUP & HOMING
    print("Enabling and homing stepper... (Press BTN_3 / trigger LIM1 to home)")
    robot.step_enable(STEPPER_LIFT)
    
    # Apply velocity and acceleration settings defined at the top
    robot.step_set_config(
        STEPPER_LIFT, 
        max_velocity=STEPPER_MAX_VELOCITY, 
        acceleration=STEPPER_ACCELERATION
    )
    
    # Home the motor first so it has a safe starting reference
    #homed = robot.step_home(
        #STEPPER_LIFT,
        #direction=-1,
        #home_velocity=STEPPER_HOME_VELOCITY,
        #backoff_steps=50,
        #blocking=True,
        #timeout=15.0,
    #)
    
    #if not homed:
        #print("Homing failed! Exiting test script.")
        #robot.step_disable(STEPPER_LIFT)
        #return

    # 2. SERVO SETUP
    print("Enabling servo...")
    robot.enable_servo(GRIPPER_SERVO)

    try:
        # --- TEST LOOP ---
        print("\nStarting test sequence. Press Ctrl+C to stop.")
        while True:
            # Test Servo: Open
            print(f"Servo: Moving to {GRIPPER_OPEN_DEG}°")
            robot.set_servo(GRIPPER_SERVO, GRIPPER_OPEN_DEG)
            time.sleep(GRIPPER_SETTLE_S)

            # Test Stepper: Retract
            # Uses a negative value to reverse the relative direction
            print(f"Stepper: Retracting backward {LIFT_STEPS} steps")
            robot.step_move(
                STEPPER_LIFT,
                steps=-LIFT_STEPS, 
                move_type=StepMoveType.RELATIVE,
                blocking=True,
                timeout=5.0
            )
            time.sleep(2.0)
            
            # Test Servo: Close
            print(f"Servo: Moving to {GRIPPER_CLOSE_DEG}°")
            robot.set_servo(GRIPPER_SERVO, GRIPPER_CLOSE_DEG)
            time.sleep(GRIPPER_SETTLE_S)

            # Test Stepper: Extend
            print(f"Stepper: Extending forward {LIFT_STEPS} steps")
            robot.step_move(
                STEPPER_LIFT,
                steps=LIFT_STEPS,
                move_type=StepMoveType.RELATIVE,
                blocking=True,
                timeout=5.0
            )
            time.sleep(1.0)

    except KeyboardInterrupt:
        print("\nTesting stopped by user. Cleaning up...")
    finally:
        # Safe shutdown of actuators
        robot.step_disable(STEPPER_LIFT)
        robot.disable_servo(GRIPPER_SERVO)
        print("Actuators disabled.")

if __name__ == "__main__":
    main()