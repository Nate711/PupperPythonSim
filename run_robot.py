import pigpio
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
import time
import numpy as np
import UDPComms


def main():
    """Main program
    """
    pi_board = pigpio.pi()
    pwm_params = PWMParams()
    servo_params = ServoParams()

    controller = Controller()
    controller.movement_reference = MovementReference()
<<<<<<< HEAD
    controller.movement_reference.v_xy_ref = np.array([0.0, 0.0])
=======
    controller.movement_reference.v_xy_ref = np.array([0.0, 0.00])
>>>>>>> a5500ce2faf081509710ba2d7317984ce0eada2f
    controller.movement_reference.wz_ref = 0
    controller.swing_params = SwingParams()
    controller.swing_params.z_clearance = 0.06
    controller.stance_params = StanceParams()
    controller.stance_params.delta_y = 0.08
    controller.gait_params = GaitParams()
    controller.gait_params.dt = 0.01

    initialize_pwm(pi_board, pwm_params)

    values = UDPComms.Subscriber(8830)
    last_loop = time.time()
    now = last_loop
    start = time.time()
    for i in range(6000):
        last_loop = time.time()
        step_controller(controller)
        send_servo_commands(pi_board, pwm_params, servo_params, controller.joint_angles)
        msg = values.get()
        controller.movement_reference.v_xy_ref = np.array([msg["x"], msg["y"]])
        controller.movement_reference.wz_ref = msg["twist"]
        while now - last_loop < controller.gait_params.dt:
            now = time.time()
        print("Time since last loop: ", now - last_loop)
    end = time.time()
    print("seconds per loop: ", (end - start) / 1000.0)


main()
