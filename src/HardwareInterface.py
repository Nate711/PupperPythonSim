import numpy as np


class PWMParams:
    def __init__(self):
        self.pins = np.array([[2, 2, 2, 2], [18, 2, 2, 2], [2, 2, 2, 2]])
        self.range = 4000
        self.freq = 250
        self.min = 800
        self.max = 2200

    @property
    def middle_pulsewidth(self):
        return (self.max + self.min) / 2.0

    @property
    def pulse_range(self):
        return self.max - self.min


class ServoParams:
    def __init__(self):
        self.neutral_position_pwm = 1500  # Middle position
        self.micros_per_rad = 1000.0 / (100.0 / 180.0 * np.pi)  # Must be calibrated

        # The neutral angle of the joint relative to the modeled zero-angle in degrees, for each joint
        self.neutral_angle_degrees = np.array(
            [[0, 0, 0, 0], [45, 45, 45, 45], [-45, -45, -45, -45]]
        )

    @property
    def neutral_angles(self):
        return self.neutral_angle_degrees * np.pi / 180.0  # Convert to radians


def pwm_to_duty_cycle(pulsewidth_micros, pwm_params):
    return int(pulsewidth_micros / 1e6 * pwm_params.freq * pwmparams.range)


def angle_to_pwm(angle, servo_params, axis_index, leg_index):
    angle_deviation = angle - servo_params.neutral_angles[axis_index, leg_index]
    pulse_width_micros = (
        servo_params.neutral_position_pwm + servo_params.micros_per_rad * angle_deviation
    )
    return pulse_width_micros


def angle_to_duty_cycle(angle, pwm_params, servo_params, axis_index, leg_index):
    return pwm_to_duty_cycle(
        angle_to_pwm(angle, servo_params, axis_index, leg_index), pwm_params
    )


def initialize_pwm(pi, pwm_params):
    for leg_index in range(4):
        for axis_index in range(3):
            pi.set_PWM_frequency(
                pwm_params.pins[axis_index, leg_index], pwm_params.freq
            )
            pi.set_PWM_range(pwm_params.pins[axis_index, leg_index], pwm_params.range)


def send_servo_commands(pi, pwm_params, servo_params, joint_angles):
    for leg_index in range(4):
        for axis_index in range(3):
            duty_cycle = angle_to_duty_cycle(
                joint_angles[axis_index, leg_index],
                pwm_params,
                servo_params,
                axis_index,
                leg_index,
            )
            pi.set_PWM_dutycycle(pwm_params.pins[axis_index, leg_index], duty_cycle)
