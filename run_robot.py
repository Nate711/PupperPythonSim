import threading
import pigpio
from src.Controller import step_controller, Controller
from src.HardwareInterface import send_servo_commands, initialize_pwm
from src.PupperConfig import (
    MovementReference,
    GaitParams,
    StanceParams,
    SwingParams,
    ServoParams,
    PWMParams,
)
import time
import numpy as np
import serial
import queue

SERIAL_PORT = "/dev/ttyACM0"
BAUD_RATE = 115200

def serial_reader(queue):
    with serial.Serial(SERIAL_PORT, BAUD_RATE) as ser:
        while True:
            ser_out = ser.readline().decode('ascii').split(" ")
            queue.put(ser_out)

def main():
    """Main program
    """
    dataQ = queue.Queue()

    serial_thread = threading.Thread(target=serial_reader, args=(dataQ,))

    serial_thread.start()

    pi_board = pigpio.pi()
    pwm_params = PWMParams()
    servo_params = ServoParams()

    controller = Controller()
    controller.movement_reference = MovementReference()
    controller.movement_reference.v_xy_ref = np.array([0.0, 0.00])
    controller.movement_reference.wz_ref = 0
    controller.swing_params = SwingParams()
    controller.swing_params.z_clearance = 0.06
    controller.stance_params = StanceParams()
    controller.stance_params.delta_y = 0.08
    controller.gait_params = GaitParams()
    controller.gait_params.dt = 0.01

    initialize_pwm(pi_board, pwm_params)

    last_loop = time.time()
    now = last_loop
    start = time.time()
    lastPos = 0;
    while True:
        last_loop = time.time()
        controller.movement_reference.wz_ref = int(-lastPos)/850
        print(lastPos)
        step_controller(controller)
        send_servo_commands(pi_board, pwm_params, servo_params, controller.joint_angles)
        while not dataQ.empty():
            print("HERE")
            lastPos = int(dataQ.get()[1])
        time.sleep(.007)

    serial_thread.join()


main()
