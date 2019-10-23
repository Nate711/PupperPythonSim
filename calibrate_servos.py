import pigpio
from src.HardwareInterface import (
    send_servo_commands,
    initialize_pwm,
    PWMParams,
    ServoParams,
    pwm_to_duty_cycle,
    send_servo_command
)
import numpy as np


def getMotorName(i, j):
    motor_type = {
        0: "Abduction",
        1: "Inner", #Top
        2: "Outer" #Bottom
    }
    leg_pos = {
        0: "Front Right",
        1: "Front Left",
        2: "Back Right",
        3: "Back Left"
    }
    final_name = motor_type[i] + " " + leg_pos[j]
    return final_name


def getMotorSetPoint(i, j):
    data = [[0, 0, 0, 0], [45, 45, 45, 45], [45, 45, 45, 45]]
    return data[i, j]


def getUserInput(request):
    measured_angle = float(input(request))
    return measured_angle


def degreesToRadians(input_array):
    return (np.pi/180.0 * input_array)


def calibrateK(servo_params, pi_board, pwm_params):
    k = np.zeros((3, 4))
    b = np.zeros((3, 4))
    offset1 = 500
    offset2 = -500
    servo_base = 1500
    for j in range(4):
        for i in range(3):
            motor_name = getMotorName(i, j)
            print("Currently calibrating " + motor_name)
            motor_instructions = np.full((3, 4), servo_base)

            # set to 1500 + some offset
            motor_instructions[i][j] = servo_base + offset1
            doty = pwm_to_duty_cycle(motor_instructions[i][j], pwm_params)
            pi_board.set_PWM_dutycycle(pwm_params.pins[i, j], doty)

            #user measures robot angle
            angle1 = getUserInput("Please measure the angle of the joint for " + motor_name)

            #set to 1500 + some other offset
            motor_instructions[i][j] = servo_base + offset2
            dty = pwm_to_duty_cycle(motor_instructions[i, j], pwm_params)
            pi_board.set_PWM_dutycycle(pwm_params.pins[i, j], dty)
            #user measures again
            angle2 = getUserInput("Please measure the angle of the joint for " + motor_name)

            s = servo_params.servo_multipliers[i][j]

            k[i][j] = (offset2 - offset1) / (s * angle1 - s * angle2)
            b[i][j] = (offset1 - s * angle1 * k[i][j]) / k[i][j]

            print("k: " + "[" + str(i) + " " + str(j) + "]", k[i,j])
            print("b: " + "[" + str(i) + " " + str(j) + "]", b[i,j])

    return k, b


def stepUntil(servo_params, pi_board, pwm_params, kValue, i_index, j_index, set_point):
    #returns the (program_angle) once the real angle matches the pre-defined set point
    
    foundPosition = False
    
    set_names = ["horizontal", "horizontal", "vertical"]
    setPointName = set_names[i_index]

    calibrated_setpoint = set_point

    while not foundPosition:
        aboveOrBelow = str(input("is the leg 'above' or 'below' " + setPointName))
        if aboveOrBelow == "above":
            calibrated_setpoint += degreesToRadians(1.0)
            send_servo_command(pi_board, pwm_params, servo_params, calibrated_setpoint, i, j)
        elif aboveOrBelow == "below":
            calibrated_setpoint -= degreesToRadians(1.0)
            send_servo_command(pi_board, pwm_params, servo_params, calibrated_setpoint, i, j)
        else:
            foundPosition = True
        print("calibrated: ", calibrated_setpoint, " orig: ", set_point)
    
    return calibrated_setpoint - set_point



def calibrateB(servo_params, pi_board, pwm_params):
    #Found K value of (11.4)
    kValue = getUserInput("Please provide a K value (microseconds per degree) for your servos: ")
    servo_params.micros_per_rad = kValue * 180/np.pi
    
    beta_values = np.zeros((3, 4))
    
    servo_params.neutral_angle_degrees = np.zeros((3, 4))
    
    for j in range(4):
        for i in range(3):
            motor_name = getMotorName(i, j)
            print("Currently calibrating " + motor_name + "...")
            set_point = getMotorSetPoint(i, j)

            # move servo to set_point angle
            send_servo_command(pi_board, pwm_params, servo_params, set_point, i, j)
            # stepuntil we like it

            offset = stepUntil(servo_params, pi_board, pwm_params, kValue, i, j, set_point)
            print("Final offset: ", offset)
            beta_values[i, j] += set_point + offset
            print("New neutral angles: ", beta_values)
    return beta_values

    #(real_angle) = s*(program_angle) - (beta)
    #(beta) = s*(program_angle) - (real_angle)

    


def main():
    """Main program
    """
    pi_board = pigpio.pi()
    pwm_params = PWMParams()
    servo_params = ServoParams()
    initialize_pwm(pi_board, pwm_params)
    # new_servo_multiplier, new_neutral_angle_degrees = calibrateK(servo_params, pi_board, pwm_params)

    # servo_params.neutral_angle_degrees = new_neutral_angle_degrees
    # servo_params.micros_per_rad = new_servo_multiplier[0]

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
