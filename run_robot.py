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
    controller.movement_reference.v_xy_ref = np.array([0.0, 0.0])
    controller.movement_reference.wz_ref = 0

    controller.movement_reference.pitch = 15.0 * np.pi / 180.0
    controller.movement_reference.roll = 0

    controller.swing_params = SwingParams()
    controller.swing_params.z_clearance = 0.06
    controller.stance_params = StanceParams()
    controller.stance_params.delta_y = 0.08
    controller.gait_params = GaitParams()
    controller.gait_params.dt = 0.01

    initialize_pwm(pi_board, pwm_params)

    values = UDPComms.Subscriber(8830, timeout=0.3)
    last_loop = time.time()
    now = last_loop
    start = time.time()
    for i in range(60000):
        last_loop = time.time()
        step_controller(controller)
        send_servo_commands(pi_board, pwm_params, servo_params, controller.joint_angles)

        try:
            msg = values.get()
            print(msg)
        except UDPComms.timeout:
            print("timout")
            msg = {"x":0, "y":0, "twist":0, "pitch":0}
        x_vel = msg["y"] / 7.0
        y_vel = -msg["x"] / 7.0
        yaw_rate = -msg["twist"] * 0.8

        pitch = msg["pitch"] * 30.0 * np.pi / 180.0
        print(pitch)

        controller.movement_reference.v_xy_ref = np.array([x_vel, y_vel])
        controller.movement_reference.wz_ref = yaw_rate
        controller.movement_reference.pitch = pitch
        while now - last_loop < controller.gait_params.dt:
            now = time.time()
        #print("Time since last loop: ", now - last_loop)
    end = time.time()
    print("seconds per loop: ", (end - start) / 1000.0)


main()
