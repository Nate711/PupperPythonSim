import pigpio
import numpy as np
import UDPComms
import time
from src.Controller import step_controller, Controller
from src.HardwareInterface import send_servo_commands, initialize_pwm
from src.PupperConfig import (
    MovementReference,
    GaitParams,
    StanceParams,
    SwingParams,
    ServoParams,
    PWMParams,
)
from src.UserInput import UserInputs, get_input, update_controller


def main():
    """Main program
    """
    pi_board = pigpio.pi()
    pwm_params = PWMParams()
    initialize_pwm(pi_board, pwm_params)

    servo_params = ServoParams()

    controller = Controller()
    controller.movement_reference = MovementReference()
    controller.movement_reference.v_xy_ref = np.array([0.0, 0.0])
    controller.movement_reference.wz_ref = 0

    controller.movement_reference.pitch = 15.0 * np.pi / 180.0
    controller.movement_reference.roll = 0

    controller.swing_params = SwingParams()
    controller.swing_params.z_clearance = 0.06
    controller.stance_params = StanceParams()
    controller.stance_params.delta_y = 0.08
    controller.gait_params = GaitParams()

    user_input = UserInputs()

    last_loop = time.time()
    now = last_loop
    start = time.time()

    for i in range(60000):
        last_loop = time.time()
        
        # Parse the udp joystick commands and then update the robot controller's parameters
        get_input(user_input)
        update_controller(controller, user_input)

        # Step the controller forward by dt
        step_controller(controller)

        # Update the pwm widths going to the servos
        send_servo_commands(pi_board, pwm_params, servo_params, controller.joint_angles)

        # Wait until it's time to execute again
        while now - last_loop < controller.gait_params.dt:
            now = time.time()
        # print("Time since last loop: ", now - last_loop)

    end = time.time()
    print("seconds per loop: ", (end - start) / 1000.0)


main()
