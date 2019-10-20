import pigpio
from src.HardwareInterface import (
    send_servo_commands,
    initialize_pwm,
    PWMParams,
    ServoParams,
    pwm_to_duty_cycle
)
import numpy as np

def getMotorName(i, j):
    motor_type = {
        0: "Abduction",
        1: "Top",
        2: "Bottom"
    }
    leg_pos = {
        0: "Front Right",
        1: "Front Left",
        2: "Back Right",
        3: "Back Left"
    }
    final_name = motor_type[i] + " " + leg_pos[j]
    return final_name


def getUserInput(request):
    measured_angle = int(input(request))
    return measured_angle


def calibrateK(servo_params):
    k = np.zeros((3, 4))
    b = np.zeros((3, 4))
    offset1 = 500
    offset2 = -500
    servo_base = 1500
    for j in range(4):
        for i in range(3):
            motor_name = getMotorName(i, j)
            print("Currently calibrating " + motor_name)
            motor_instructions = np.full((3, 4), 1500)

            # set to 1500 + some offset
            motor_instructions[i][j] = offset1
            pwm_to_duty_cycle(motor_instructions)

            #user measures robot angle
            angle1 = getUserInput("Please measure the angle of the joint for " + motor_name)

            #set to 1500 + some other offset
            motor_instructions[i][j] = offset2
            pwm_to_duty_cycle(motor_instructions)

            #user measures again
            angle2 = getUserInput("Please measure the angle of the joint for " + motor_name)

            s = servo_params.servo_multipliers[i][j]

            k[i][j] = (offset2 - offset1) / (s * angle1 - s * angle2)
            b[i][j] = (offset1 - s * angle1 * k[i][j]) / k[i][j]

    return k, b


def main():
    """Main program
    """
    pi_board = pigpio.pi()
    pwm_params = PWMParams()
    servo_params = ServoParams()
    initialize_pwm(pi_board, pwm_params)
    new_servo_multiplier, new_neutral_angle_degrees = calibrateK(servo_params)

    servo_params.neutral_angle_degrees = new_neutral_angle_degrees
    servo_params.micros_per_rad = new_servo_multiplier[0]

    """
    servo_params.neutral_angle_degrees = np.array(
        [[8, 3, 0, 0], [45, 48, 45, 45], [-50, -38, -45, -45]]
    )

    ref_position = np.pi/180.0 * np.array([[0, 0, 0, 0], [0, 0, 45, 45], [-45,-45, -45, -45]])
    send_servo_commands(pi_board, pwm_params, servo_params, ref_position)
    """



main()

self.servo_multipliers = np.array(
    [[1, 1, 1, 1], [-1, 1, 1, -1], [1, -1, 1, -1]]
)