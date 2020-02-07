import pigpio
import numpy as np
import UDPComms
import time
import subprocess
from src.IMU import IMU
from src.Controller import step_controller, Controller
from src.HardwareInterface import send_servo_commands, initialize_pwm
from src.PupperConfig import (
    PupperConfig,
    MovementReference,
    GaitParams,
    StanceParams,
    SwingParams,
    ServoParams,
    PWMParams,
    UserInputParams,
)
from src.UserInput import UserInputs, get_input, update_controller


def start_pigpiod():
    """Does not seem to work!"""
    subprocess.Popen(["sudo", "pigpiod"])
    time.sleep(2)

def stop_pigpiod():
    subprocess.Popen(["sudo", "pkill", "pigpiod"])

def main():
    """Main program
    """
    print("Starting pigpiod...")
    start_pigpiod()
    print("Done.")

    # Start pwm to servos
    pi_board = pigpio.pi()
    pwm_params = PWMParams()
    initialize_pwm(pi_board, pwm_params)

    # Create config
    robot_config = PupperConfig()
    servo_params = ServoParams()

    # Create imu handle
    imu = IMU(port="/dev/ttyACM0")
    imu.flush_buffer()

    # Create controller and user input handles
    controller = Controller(robot_config)
    input_params = UserInputParams()
    user_input = UserInputs(max_x_velocity=input_params.max_x_velocity, max_y_velocity=input_params.max_y_velocity, max_yaw_rate=input_params.max_yaw_rate, max_pitch=input_params.max_pitch)

    last_loop = time.time()

    print("Summary of gait parameters:")
    print("overlap time: ", controller.gait_params.overlap_time)
    print("swing time: ", controller.gait_params.swing_time)
    print("z clearance: ", controller.swing_params.z_clearance)
    print("x shift: ", controller.stance_params.x_shift)

    # Wait until the activate button has been pressed
    while True:
        get_input(user_input)
        if user_input.activate:
            user_input.last_activate = 1
            break

    while True:
        now = time.time()
        if now - last_loop < controller.gait_params.dt:
            continue
        last_loop = time.time()

        # Parse the udp joystick commands and then update the robot controller's parameters
        get_input(user_input)
        if user_input.activate == 1 and user_input.last_activate == 0:
            break
        else:
            user_input.last_activate = user_input.activate

        update_controller(controller, user_input)

        # Read imu data. Orientation will be None if no data was available
        quat_orientation = imu.read_orientation()

        # Step the controller forward by dt
        step_controller(controller, robot_config, quat_orientation)

        # Update the pwm widths going to the servos
        send_servo_commands(pi_board, pwm_params, servo_params, controller.joint_angles)
    
    print("Killing pigpiod...")
    stop_pigpiod()
    print("Done.")

main()
