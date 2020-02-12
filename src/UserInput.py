import UDPComms
import numpy as np

class UserInputs:
    def __init__(self, max_x_velocity, max_y_velocity, max_yaw_rate, max_pitch, udp_port=8830):
        self.max_x_velocity = max_x_velocity
        self.max_y_velocity = max_y_velocity
        self.max_yaw_rate = max_yaw_rate
        self.max_pitch = max_pitch

        self.x_vel = 0.0
        self.y_vel = 0.0
        self.yaw_rate = 0.0
        self.pitch = 0.0

        self.stance_movement = 0
        self.roll_movement = 0

        self.gait_toggle = 0
        self.gait_mode = 0
        self.previous_state = 0
        self.current_state = 0

        self.activate = 0
        self.last_activate = 0

        self.message_rate = 50
        self.udp_handle = UDPComms.Subscriber(udp_port, timeout=0.3)


def get_input(user_input_obj, do_print=False):
    try:
        msg = user_input_obj.udp_handle.get()
        user_input_obj.x_vel = msg["ly"] * 0.5
        user_input_obj.y_vel = msg["lx"] * -0.24
        user_input_obj.yaw_rate = msg["rx"] * -2.0
        user_input_obj.pitch = msg["ry"] * 30 * np.pi / 180.0
        user_input_obj.gait_toggle = msg["R1"]
        user_input_obj.activate = msg["L1"]
        user_input_obj.stance_movement = msg["dpady"]
        user_input_obj.roll_movement = msg["dpadx"]
        user_input_obj.message_rate = msg["message_rate"]
        user_input_obj.hop_toggle = msg["x"]

        # Update gait mode
        if user_input_obj.previous_state == 0:
            if user_input_obj.gait_toggle == 1:
                user_input_obj.current_state = 2
        if user_input_obj.hop_toggle == 1:
            user_input_obj.current_state = 1
            
        user_input_obj.previous_state = user_input_obj.current_state

    except UDPComms.timeout:
        if do_print:
            print("UDP Timed out")


def update_controller(controller, user_input_obj):
    controller.movement_reference.v_xy_ref = np.array(
        [user_input_obj.x_vel, user_input_obj.y_vel]
    )
    controller.movement_reference.wz_ref = user_input_obj.yaw_rate

    message_dt = 1.0 / user_input_obj.message_rate
    alpha = message_dt / controller.stance_params.pitch_time_constant
    controller.movement_reference.pitch = (
        controller.movement_reference.pitch * (1 - alpha) + user_input_obj.pitch * alpha
    )

    if user_input_obj.gait_mode == 0:
        controller.gait_params.contact_phases = np.array(
            [[1, 1, 1, 1], [1, 1, 1, 1], [1, 1, 1, 1], [1, 1, 1, 1]]
        )
    else:
        controller.gait_params.contact_phases = np.array(
            [[1, 1, 1, 0], [1, 0, 1, 1], [1, 0, 1, 1], [1, 1, 1, 0]]
        )
        #0 is trotting
        #1 is hopping 
        #2 is resting
    if user_input_obj.hop == 1:
        controller.special_movement = 1
    # Note this is negative since it is the feet relative to the body
    controller.movement_reference.z_ref -= (
        controller.stance_params.z_speed * message_dt * user_input_obj.stance_movement
    )
    controller.movement_reference.roll += (
        controller.stance_params.roll_speed * message_dt * user_input_obj.roll_movement
    )
