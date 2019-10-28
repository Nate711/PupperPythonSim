import pigpio
from src.HardwareInterface import (
    send_servo_commands,
    initialize_pwm,
    pwm_to_duty_cycle,
    send_servo_command,
)
from src.PupperConfig import PWMParams, ServoParams
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

    offset = 0

    while not foundPosition:
        aboveOrBelow = str(
            input("Is the leg 'above' or 'below' " + set_names[i_index]+ "? Input: ")
        )
        if aboveOrBelow == "above" or aboveOrBelow == "a":
            offset += 1.0
            send_servo_command(
                pi_board,
                pwm_params,
                servo_params,
                degreesToRadians(set_point + offset),
                i_index,
                j_index,
            )
        elif aboveOrBelow == "below" or aboveOrBelow == "b":
            offset -= 1.0
            send_servo_command(
                pi_board,
                pwm_params,
                servo_params,
                degreesToRadians(set_point + offset),
                i_index,
                j_index,
            )
        elif aboveOrBelow == "done" or aboveOrBelow == "d":
            foundPosition = True
        print("offset: ", offset, " original: ", set_point)

    return offset


def calibrateB(servo_params, pi_board, pwm_params):
    """Calibrate the angle offset for the twelve motors on the robot. Note that servo_params is modified in-place.
    Parameters
    ----------
    servo_params : ServoParams
        Servo parameters. This variable is updated in-place.
    pi_board : Pi
        RaspberryPi object.
    pwm_params : PWMParams
        PWMParams object.
    """

    # Found K value of (11.4)
    kValue = getUserInput(
        "Please provide a K value (microseconds per degree) for your servos: "
    )
    servo_params.micros_per_rad = kValue * 180 / np.pi

    servo_params.neutral_angle_degrees = np.zeros((3, 4))

    for j in range(4):
        for i in range(3):
            # Loop until we're satisfied with the calibration
            completed = False
            while not completed:
                motor_name = getMotorName(i, j)
                print("Currently calibrating " + motor_name + "...")
                set_point = getMotorSetPoint(i, j)

                # Move servo to set_point angle
                send_servo_command(
                    pi_board,
                    pwm_params,
                    servo_params,
                    degreesToRadians(set_point),
                    i,
                    j,
                )

                # Adjust the angle using keyboard input until it matches the reference angle
                offset = stepUntil(
                    servo_params, pi_board, pwm_params, kValue, i, j, set_point
                )
                print("Final offset: ", offset)

                # The upper leg link has a different equation because we're calibrating to make it horizontal, not vertical
                if i == 1:
                    servo_params.neutral_angle_degrees[i, j] = set_point - offset
                else:
                    servo_params.neutral_angle_degrees[i, j] = -(set_point + offset)
                print("New beta angle: ", servo_params.neutral_angle_degrees[i, j])

                # Send the servo command using the new beta value and check that it's ok
                send_servo_command(
                    pi_board,
                    pwm_params,
                    servo_params,
                    degreesToRadians([0, 45, -45][i]),
                    i,
                    j,
                )
                okay = ""
                while okay not in ["yes", "no"]:
                    okay = str(
                        input("Check angle. Are you satisfied? Enter 'yes' or 'no']")
                    )
                completed = okay == "yes"

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
    print("Calibrated neutral angles:")
    print(servo_params.neutral_angle_degrees)
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
