import pigpio
from src.HardwareInterface import (
    send_servo_commands,
    initialize_pwm,
    PWMParams,
    ServoParams,
)
import numpy as np


def main():
    """Main program
    """
    pi_board = pigpio.pi()
    pwm_params = PWMParams()
    servo_params = ServoParams()
    initialize_pwm(pi_board, pwm_params)

    servo_params.neutral_angle_degrees = np.array(
        [[7, 5, 0, 0], [45, 48, 45, 45], [-50, -38, -45, -45]]
    )

    ref_position = np.pi/180.0 * np.array([[-30, 30, 0, 0], [90, 90, 45, 45], [-45,-45, -45, -45]])
    send_servo_commands(pi_board, pwm_params, servo_params, ref_position)


main()
