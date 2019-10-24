import pigpio
from src.HardwareInterface import (
    send_servo_commands,
    initialize_pwm,
    PWMParams,
    ServoParams,
    pwm_to_duty_cycle,
    send_servo_command,
)
import numpy as np


def getMotorName(i, j):
    motor_type = {0: "Abduction", 1: "Inner", 2: "Outer"}  # Top  # Bottom
    leg_pos = {0: "Front Right", 1: "Front Left", 2: "Back Right", 3: "Back Left"}
    final_name = motor_type[i] + " " + leg_pos[j]
    return final_name


def getMotorSetPoint(i, j):
    data = [[0, 0, 0, 0], [45, 45, 45, 45], [45, 45, 45, 45]]
    return data[i][j]


def getUserInput(request):
    measured_angle = float(input(request))
    return measured_angle


def degreesToRadians(input_array):
    return np.pi / 180.0 * input_array


def stepUntil(servo_params, pi_board, pwm_params, kValue, i_index, j_index, set_point):
    # returns the (program_angle) once the real angle matches the pre-defined set point
    foundPosition = False
    set_names = ["horizontal", "horizontal", "vertical"]
    calibrated_setpoint = set_point

    while not foundPosition:
        aboveOrBelow = str(
            input("is the leg 'above' or 'below' or 'done': " + set_names[i_index])
        )
        if aboveOrBelow == "above" or aboveOrBelow == "a":
            calibrated_setpoint += 1.0
            send_servo_command(
                pi_board,
                pwm_params,
                servo_params,
                degreesToRadians(calibrated_setpoint),
                i_index,
                j_index,
            )
        elif aboveOrBelow == "below" or aboveOrBelow == "b":
            calibrated_setpoint -= 1.0
            send_servo_command(
                pi_board,
                pwm_params,
                servo_params,
                degreesToRadians(calibrated_setpoint),
                i_index,
                j_index,
            )
        elif aboveOrBelow == "done" or "d":
            foundPosition = True
        print("calibrated: ", calibrated_setpoint, " orig: ", set_point)

    return calibrated_setpoint - set_point


def calibrateB(servo_params, pi_board, pwm_params):
    # Found K value of (11.4)
    kValue = getUserInput(
        "Please provide a K value (microseconds per degree) for your servos: "
    )
    servo_params.micros_per_rad = kValue * 180 / np.pi

    beta_values = np.zeros((3, 4))

    servo_params.neutral_angle_degrees = np.zeros((3, 4))

    for j in range(4):
        for i in range(3):
            motor_name = getMotorName(i, j)
            print("Currently calibrating " + motor_name + "...")
            set_point = getMotorSetPoint(i, j)

            # move servo to set_point angle
            send_servo_command(
                pi_board, pwm_params, servo_params, degreesToRadians(set_point), i, j
            )
            # stepuntil we like it

            offset = stepUntil(
                servo_params, pi_board, pwm_params, kValue, i, j, set_point
            )
            print("Final offset: ", offset)
            # The original line below is actually incorrect. Let's talk about it later, hard to explain in words
            # The actual lines should be something like:
            # if i == 2:
            #     offset = 90 - offset
            # beta_values[i, j] = default_offset + offset
            #beta_values[i, j] += set_point + offset
            #print("New neutral angles: ", beta_values)

    # return beta_values

    # (real_angle) = s*(program_angle) - (beta)
    # (beta) = s*(program_angle) - (real_angle)


def main():
    """Main program
    """
    pi_board = pigpio.pi()
    pwm_params = PWMParams()
    servo_params = ServoParams()
    initialize_pwm(pi_board, pwm_params)

    calibrateB(servo_params, pi_board, pwm_params)
    """
    servo_params.neutral_angle_degrees = np.array(
        [[8, 3, 0, 0], [45, 48, 45, 45], [-50, -38, -45, -45]]
    )

    ref_position = np.pi/180.0 * np.array([[0, 0, 0, 0], [0, 0, 45, 45], [-45,-45, -45, -45]])
    send_servo_commands(pi_board, pwm_params, servo_params, ref_position)
    """


main()

# self.servo_multipliers = np.array(
#     [[1, 1, 1, 1], [-1, 1, 1, -1], [1, -1, 1, -1]]
# )
