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
        [[0, 0, 0, 0], [45, 45, 45, 45], [-45, -45, -45, -45]]
    )

    ref_position = servo_params.neutral_angles
    send_servo_commands(pi_board, pwm_params, servo_params, ref_position)


main()
