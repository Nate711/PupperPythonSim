import numpy as np
from PiGPIO import pi, set_PWM_dutycycle, set_PWM_frequency, set_PWM_range


class PWMParams:
    def __init__(self):
        self.pins = np.array([2, 18, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2])
        self.range = 4000
        self.freq = 250
        self.min = 1000
        self.max = 2000
        self.middle_pulsewidth = (self.max + self.min) / 2.0
        self.pulse_range = self.max - self.min


class ServoParams:
    def __init__(self):
        self.angle_range = 160 / 180.0 * pi
        self.neutral_range = 0


def angle_to_duty_cycle(angle, pwm_params, servo_params):
    normalized_angle = (angle - servo_params.neutral_range) / servo_params.angle_range
    pulse_width_micros = (
        normalized_angle * pwm_params.pulse_range + pwm_params.middle_pulsewidth
    )
    return pulse_width_micros


def initialize_pwm(pi, pwm_params):
    for leg_index in range(4):
        for axis_index in range(3):
            set_PWM_frequency(
                pi, pwm_params.pins[axis_index, leg_index], pwm_params.freq
            )
            set_PWM_range(pi, pwm_params.pins[axis_index, leg_index, pwm_params.range])


def send_servo_commands(pi, pwm_params, servo_params, joint_angles):
    for leg_index in range(4):
        for axis_index in range(3):
            duty_cycle = angle_to_duty_cycle(
                joint_angles[axis_index, leg_index], pwm_params, servo_params
            )
            set_PWM_dutycycle(pi, pwm_params[axis_index, leg_index], duty_cycle)
