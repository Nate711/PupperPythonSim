
import pigpio
from src.Controller import step_controller, Controller
from src.HardwareInterface import (
    send_servo_commands,
    initialize_pwm,
    PWMParams,
    ServoParams,
)
from src.PupperConfig import MovementReference, GaitParams, StanceParams, SwingParams
import time
import numpy as np


def main():
    """Main program
    """
    pi_board = pigpio.pi()
    pwm_params = PWMParams()
    servo_params = ServoParams()

    controller = Controller()
    controller.movement_reference = MovementReference()
    controller.movement_reference.v_xy_ref = np.array([0.0, 0.15])
    controller.movement_reference.wz_ref = 0
    controller.swing_params = SwingParams()
    controller.swing_params.z_clearance = 0.02
    controller.stance_params = StanceParams()
    controller.stance_params.delta_y = 0.06
    controller.gait_params = GaitParams()
    controller.gait_params.dt = 0.01

    initialize_pwm(pi_board, pwm_params)

    last_loop = time.time()
    now = last_loop
    start = time.time()
    for i in range(6000):
        last_loop = time.time()
        step_controller(controller)
        send_servo_commands(pi_board, pwm_params, servo_params, controller.joint_angles)

        while now - last_loop < controller.gait_params.dt:
            now = time.time()
        # print("Time since last loop: ", now - last_loop)
    end = time.time()
    print("seconds per loop: ", (end - start) / 1000.0)


main()
