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


def set_velocity(controller, velocity_x, velocity_y):
    controller.movement_reference.v_xy_ref = np.array([velocity_x, velocity_y])
    controller.movement_reference.wz_ref = 0


def turn_radians(pi_board, pwm_params, servo_params, controller, speed, radians):
    time_to_run = radians / speed
    turn_for_time(pi_board, pwm_params, servo_params, controller, speed, time_to_run)


def turn_degrees(pi_board, pwm_params, servo_params, controller, speed, degrees):
    turn_radians(pi_board, pwm_params, servo_params, controller, speed, degrees * np.pi / 180)


def turn_for_time(pi_board, pwm_params, servo_params, controller, speed, time):
    start_time = time.time()
    while time.time() - start_time < time:
        controller.movement_reference.v_xy_ref = np.array([0.0, 0.00])
        controller.movement_reference.wz_ref = speed
        step_controller(controller)
        send_servo_commands(pi_board, pwm_params, servo_params, controller.joint_angles)


def main():
    """Main program
    """
    pi_board = pigpio.pi()
    pwm_params = PWMParams()
    servo_params = ServoParams()

    controller = Controller()
    controller.movement_reference = MovementReference()
    controller.movement_reference.v_xy_ref = np.array([0.0, 0.00])
    controller.movement_reference.wz_ref = 0 #given in radians per second
    controller.swing_params = SwingParams()
    controller.swing_params.z_clearance = 0.06
    controller.stance_params = StanceParams()
    controller.stance_params.delta_y = 0.08
    controller.gait_params = GaitParams()
    controller.gait_params.dt = 0.01

    initialize_pwm(pi_board, pwm_params)

    last_loop = time.time()
    now = last_loop
    start = time.time()
    values = UDPComms.Subscriber(8870)
    #msg = {"command": "move_forward", "time": "10"}  # simulated move command
    #msg = {"command": "turn", "speed": "0.1", "radians": 1.0}  # simulated turn command
    for i in range(6000):
        last_loop = time.time()
        #get some info from UDP
        try:
            msg = values.get()
            command = msg["command"]
        except UDPComms.timeout:
            #no commands received yet
            command = "none"


        if command == "set_velocity":
            set_velocity(controller, msg["velocity_x"], msg["velocity_y"])
        elif command == "turn_radian":
            turn_radians(pi_board, pwm_params, servo_params, controller, msg["speed"], msg["radians"])
        elif command == "turn_degrees":
            turn_radians(pi_board, pwm_params, servo_params, controller, msg["speed"], msg["degrees"])

        step_controller(controller)
        send_servo_commands(pi_board, pwm_params, servo_params, controller.joint_angles)
        if i is 100:
            msg = {"command": "turn_radian", "speed": "0.1", "radians": 1.0}
        if i is 101:
            msg = {}
        while now - last_loop < controller.gait_params.dt:
            now = time.time()
        print("Time since last loop: ", now - last_loop)
    end = time.time()
    print("seconds per loop: ", (end - start) / 1000.0)


main()
