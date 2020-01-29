import pigpio
import numpy as np
import UDPComms
import time
import subprocess
from src.IMU import create_imu_handle, read_orientation
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
    IMUParams,
)
from src.UserInput import UserInputs, get_input, update_controller


def start_pigpiod():
    subprocess.Popen(["sudo", "pkill", "pigpiod"])
    subprocess.Popen(["sudo", "pigpiod"])


def main():
    """Main program
    """

    # Start pwm to servos
    # start_pigpiod()
    pi_board = pigpio.pi()
    pwm_params = PWMParams()
    initialize_pwm(pi_board, pwm_params)

    # Create config
    robot_config = PupperConfig()
    servo_params = ServoParams()

    # Create imu handle
    imu_params = IMUParams("/dev/ttyACM0")
    imu_handle = create_imu_handle(imu_params)
    imu_handle.reset_input_buffer()

    # Create controller and user input handles
    controller = Controller(robot_config)
    user_input = UserInputs()

    last_loop = time.time()

    while True:
        if time.time() - last_loop < controller.gait_params.dt:
            continue
        last_loop = time.time()

        # Parse the udp joystick commands and then update the robot controller's parameters
        get_input(user_input)
        update_controller(controller, user_input)

        # Read imu data. Orientation will be None if no data was available
        quat_orientation = read_orientation(imu_handle)
        print(quat_orientation)
        if quat_orientation is None:
            quat_orientation = np.array([1, 0, 0, 0])

        # Step the controller forward by dt
        step_controller(controller, robot_config, quat_orientation)

        # Update the pwm widths going to the servos
        send_servo_commands(pi_board, pwm_params, servo_params, controller.joint_angles)


main()
