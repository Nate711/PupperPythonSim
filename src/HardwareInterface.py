import Types
from PiGPIO import pi, set_PWM_dutycycle, set_PWM_frequency, set_PWM_range

# include("Types.jl")
# using PiGPIO

# @with_kw struct PWMParams
#     pins::Matrix{Int} = reshape([2,18,2,2,2,2,2,2,2,2,2,2], 3, 4)
#     range::Int = 4000
#     frequency::Int = 250 # Hz
#     min::Int = 1000 # minimum servo pulse width [us]
#     max::Int = 2000 # maximum servo pulse width [us]
#     middlepulsewidth::Int = (min + max) / 2
#     pulserange::Int = max - min
# end


class PWMParams():
    def __init__(self):
        self.pins = np.array([2, 18, 2,     2, 2, 2,    2, 2, 2,    2, 2, 2])
        self.range = 4000
        self.freq = 250
        self.min = 1000
        self.max = 2000
        self.middle_pulsewidth = (self.max + self.min) / 2.0
        self.pulse_range = self.max - self.min

# @with_kw struct ServoParams
#     anglerange::Float64 = 160.0 / 180.0 * pi
#     neutralangle::Float64 = 0.0
# end


class ServoParams():
    def __init__(self):
        self.angle_range = 160/180. * pi
        self.neutral_range = 0

# function angle2dutycycle(angle::Float64, pwmparams::PWMParams, servoparams::ServoParams)
#     normalizedangle = (angle - servoparams.neutralangle) / servoparams.anglerange
#     pulsewidth_micros = normalizedangle * pwmparams.pulserange + pwmparams.middlepulsewidth
#     dutycycle = Int(round(pulsewidth_micros / 1e6 * pwmparams.frequency * pwmparams.range))
#     #println(pulsewidth_micros, " ", dutycycle)
#     return dutycycle
# end


def angle_to_duty_cycle(angle, pwm_params, servo_params):
    normalized_angle = (angle - servo_params.neutral_range) / servo_params.angle_range
    pulse_width_micros = normalized_angle * pwm_params.pulse_range + pwm_params.middle_pulsewidth

# function initializePWM(pi::Pi, pwmparams::PWMParams)
#     for legindex in 1:4
#         for axis in 1:3
#             set_PWM_frequency(pi, pwmparams.pins[axis, legindex], pwmparams.frequency)
#             set_PWM_range(pi, pwmparams.pins[axis, legindex], pwmparams.range)
#         end
#     end
# end


def initialize_pwm(pi, pwm_params):
    for leg_index in range(4):
        for axis_index in range(3):
            set_PWM_frequency(pi, pwm_params.pins[axis_index, leg_index], pwm_params.freq)
            set_PWM_range(pi, pwm_params.pins[axis, leg_index, pwm_params.range)])


# def sendservocommands(pi::Pi, pwmparams::PWMParams, servoparams::ServoParams, jointangles::SMatrix{3, 4, Float64, 12})
#     for legindex in 1:4
#         for axis in 1:3
#             if legindex == 1 && axis == 2
#                 # println(jointangles[axis,legindex])
#             end
#             dutycycle = angle2dutycycle(jointangles[axis, legindex], pwmparams, servoparams)
#             set_PWM_dutycycle(pi, pwmparams.pins[axis, legindex], dutycycle)
#         end
#     end
# end


def send_servo_commands(pi, pwm_params, servo_params, joint_angle):
    for leg_index in range(4):
        for axis_index in range(3):
            duty_cycle = angle_to_duty_cycle(joint_angles[axis_index, leg_index], pwm_params, servo_params)
            set_PWM_duty(pi, pwm_params[axis_index, leg_index))
