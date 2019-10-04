import pigpio
from src.Controller import step_controller, Controller
from src.HardwareInterface import (
    send_servo_commands,
    initialize_pwm,
    PWMParams,
    ServoParams,
)
from src.Types import MovementReference, GaitParams, StanceParams, SwingParams
import time

# function loop(pi::Pi, pwmparams::PWMParams, servoparams, controller::Controller)
#     stepcontroller!(controller)
#     sendservocommands(pi, pwmparams, servoparams, controller.jointangles)
# end


def loop(pi, pwm_params, servo_params, controller):
    """[summary]
    
    Parameters
    ----------
    pi : [type]
        [description]
    pwm_params : [type]
        [description]
    servo_params : [type]
        [description]
    controller : [type]
        [description]
    """
    step_controller(controller)
    send_servo_commands(pi, pwm_params, servo_params, controller.joint_angles)


# function main()
#     piboard = Pi()
#     pwmparams = PWMParams()
#     servoparams = ServoParams()

#     controller = Controller()
#     controller.mvref = MovementReference(vxyref=SVector(0.2,0.0), zref=-0.15, wzref=0.3)
#     controller.swingparams = SwingParams(zclearance=0.02)
#     controller.stanceparams = StanceParams(Δx=0.1, Δy=0.09)
#     controller.gaitparams = GaitParams(dt=0.01)

#     initializePWM(piboard, pwmparams)

#     lastloop = Dates.Time(Dates.now())
#     now = lastloop
#     for i in 1:1000
#         stepcontroller!(controller)
#         sendservocommands(piboard, pwmparams, servoparams, controller.jointangles)

#         while now - lastloop < Nanosecond(Int(round(1e9*controller.gaitparams.dt)))
#             now = Dates.Time(Dates.now())
#         end
# 	    println("Since last loop: ", (now - lastloop))
#         lastloop = now
#     end
# end


def main():
    """[summary]
    """
    pi_board = pigpio.pi()
    pwm_params = PWMParams()
    servo_params = ServoParams()

    controller = Controller()
    controller.movement_reference = MovementReference()
    controller.movement_reference.v_xy_ref = np.array([0.2, 0])
    controller.swing_params = SwingParams()
    controller.swing_params.z_clearance = 0.02
    controller.stance_params = StanceParams()
    controller.gait_params = GaitParams()
    controller.gait_params.dt = 0.01

    initialize_pwm(pi_board, pwm_params)

    last_loop = time.time()
    now = last_loop
    for i in range(1000):
        step_controller(controller)
        send_servo_commands(pi_board, pwm_params, servo_params, controller.joint_angles)

        while now - last_loop < controller.gait_params.dt:
            now = time.time()
        print("Time since last loop: ", now - last_loop)
