# using StaticArrays

# include("Types.jl")
# include("Gait.jl")
# include("StanceController.jl")
# include("SwingLegController.jl")
# include("Kinematics.jl")
# include("PupperConfig.jl")

from Types import SwingParams, StanceParams, GaitParams, MovementReference
from PupperConfig import PupperConfig
from Gaits import contacts, subphase_time
from Kinematics import four_legs_inverse_kinematics
from StanceController import stance_foot_location
from SwingLegController import swing_foot_location

import numpy as np


# @with_kw mutable struct Controller
#     swingparams::SwingParams = SwingParams()
#     stanceparams::StanceParams = StanceParams()
#     gaitparams::GaitParams = GaitParams()
#     mvref::MovementReference = MovementReference(vxyref=SVector{2}(0.0, 0.0), wzref=0.0, zref=-0.18)
#     conparams::ControllerParams = ControllerParams()
#     robotconfig::PupperConfig = PupperConfig()
#     ticks::Int = 0

#     footlocations::SMatrix{3, 4, Float64, 12} = stanceparams.defaultstance .+ SVector{3, Float64}(0, 0, mvref.zref)
#     jointangles::SMatrix{3, 4, Float64, 12} = fourlegs_inversekinematics(footlocations, robotconfig)
# end


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


# function step(ticks::Integer, footlocations::SMatrix{3, 4, Float64}, swingparams::SwingParams, stanceparams::StanceParams, gaitparams::GaitParams, mvref::MovementReference, conparams::ControllerParams)
#     #=
#     Return the foot locations for the next timestep.
#     Allocation-free.

#     ticks: Time since the system started
#     footlocations:: SMatrix
#     =#

#     contactmodes = contacts(ticks, gaitparams)
#     for legindex in 1:4
#         contactmode = contactmodes[legindex]
#         footloc = SVector{3}(footlocations[:, legindex])
#         if contactmode == 1
#             newloc = stancefootlocation(footloc, stanceparams, gaitparams, mvref)
#         else
#             swingprop::Float64 = subphasetime(ticks, gaitparams) / gaitparams.swingticks
#             newloc = swingfootlocation(swingprop, footloc, legindex, swingparams, stanceparams, gaitparams, mvref)
#         end

#         for j in 1:3
#             footlocations = setindex(footlocations, newloc[j], LinearIndices(footlocations)[j, legindex])
#         end
#     end
#     return footlocations
# end


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
    print(ticks, contact_modes)
    print("")
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


# function stepcontroller!(c::Controller)
#     c.footlocations = step(c.ticks, c.footlocations, c.swingparams, c.stanceparams, c.gaitparams, c.mvref, c.conparams)
#     c.jointangles = fourlegs_inversekinematics(c.footlocations, c.robotconfig)
#     c.ticks += 1
# end


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


# function run()
#     c = Controller()
#     c.mvref = MovementReference(vxyref=SVector{2}(0.2, 0.0), wzref=0.5)

#     tf = 6.0
#     timesteps = Int(tf / (c.gaitparams.dt))

#     footlochistory = zeros(3, 4, timesteps)
#     jointanglehistory = zeros(3, 4, timesteps)

#     for i in 1:timesteps
#         stepcontroller!(c)
#         footlochistory[:, :, i] = c.footlocations
#         jointanglehistory[:, :, i] = c.jointangles
#     end
#     return footlochistory, jointanglehistory
# end


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
