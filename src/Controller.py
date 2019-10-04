from src.PupperConfig import SwingParams, StanceParams, GaitParams, MovementReference
from src.PupperConfig import PupperConfig
from src.Gaits import contacts, subphase_time
from src.Kinematics import four_legs_inverse_kinematics
from src.StanceController import stance_foot_location
from src.SwingLegController import swing_foot_location

import numpy as np


class Controller:
    def __init__(self):
        self.swing_params = SwingParams()
        self.stance_params = StanceParams()
        self.gait_params = GaitParams()
        self.movement_reference = MovementReference()
        self.robot_config = PupperConfig()

        self.ticks = 0

        self.foot_locations = (
            self.stance_params.default_stance
            + np.array([0, 0, self.movement_reference.z_ref])[:, np.newaxis]
        )
        self.joint_angles = four_legs_inverse_kinematics(
            self.foot_locations, self.robot_config
        )


def step(
    ticks, foot_locations, swing_params, stance_params, gait_params, movement_reference
):
    """[summary]
    
    Parameters
    ----------
    ticks : [type]
        [description]
    foot_locations : [type]
        [description]
    swing_params : [type]
        [description]
    stance_params : [type]
        [description]
    gait_params : [type]
        [description]
    movement_reference : [type]
        [description]
    
    Returns
    -------
    [type]
        [description]
    """
    contact_modes = contacts(ticks, gait_params)
    new_foot_locations = np.zeros((3, 4))
    for leg_index in range(4):
        contact_mode = contact_modes[leg_index]
        foot_location = foot_locations[:, leg_index]
        if contact_mode == 1:
            new_location = stance_foot_location(
                foot_location, stance_params, gait_params, movement_reference
            )
        else:
            swing_proportion = (
                subphase_time(ticks, gait_params) / gait_params.swing_ticks
            )
            new_location = swing_foot_location(
                swing_proportion,
                foot_location,
                leg_index,
                swing_params,
                stance_params,
                gait_params,
                movement_reference,
            )
        new_foot_locations[:, leg_index] = new_location
    return new_foot_locations


def step_controller(controller):
    """[summary]
    
    Parameters
    ----------
    controller : [type]
        [description]
    """
    controller.foot_locations = step(
        controller.ticks,
        controller.foot_locations,
        controller.swing_params,
        controller.stance_params,
        controller.gait_params,
        controller.movement_reference,
    )
    controller.joint_angles = four_legs_inverse_kinematics(
        controller.foot_locations, controller.robot_config
    )
    controller.ticks += 1


def run():
    """[summary]
    
    Returns
    -------
    [type]
        [description]
    """
    c = Controller()
    c.movement_reference.v_xy_ref = np.array([0.2, 0.0])
    c.movement_reference.wz_ref = 0.5

    tf = 1.0
    time_steps = int(tf / c.gait_params.dt)

    foot_loc_history = np.zeros((3, 4, time_steps))
    joint_angle_history = np.zeros((3, 4, time_steps))

    for i in range(time_steps):
        step_controller(c)
        foot_loc_history[:, :, i] = c.foot_locations
        joint_angle_history[:, :, i] = c.joint_angles

    return foot_loc_history, joint_angle_history
