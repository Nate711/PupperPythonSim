# using Rotations
# using LinearAlgebra
# using StaticArrays

# include("Types.jl")

import numpy as np
from transforms3d.euler import euler2mat

# function skiincrement(zmeas::Float64, stanceparams::StanceParams, mvref::MovementReference, gaitparams::GaitParams)
#     #=
#     Given a desired positioning targets, find the discrete change in position and
#     orientation to the ski for the given timestep duration.

#     zmeas: actual body z coordinate
#     stanceparams: stance controller parameters
#     gaitparams: gait controller parameters
#     mvref: movement reference
#     =#

#     Δp = SVector(-mvref.vxyref[1], -mvref.vxyref[2], 1 / stanceparams.ztimeconstant * (mvref.zref - zmeas)) * gaitparams.dt
#     ΔR = RotZ(-mvref.wzref * gaitparams.dt)
#     return (Δp, ΔR)
# end


def ski_increment(z_measured, stance_params, movement_reference, gait_params):
    """[summary]
    
    Parameters
    ----------
    z_measured : [type]
        [description]
    stance_params : [type]
        [description]
    movement_reference : [type]
        [description]
    gait_params : [type]
        [description]
    """
    v_xy = np.array(
        [
            -movement_reference.v_xy_ref[0],
            -movement_reference.v_xy_ref[1],
            1.0
            / stance_params.z_time_constant
            * (movement_reference.z_ref - z_measured),
        ]
    )
    delta_p = v_xy * gait_params.dt
    delta_R = euler2mat(0, 0, -movement_reference.wz_ref * gait_params.dt)
    return (delta_p, delta_R)


# function stancefootlocations(stancefootlocations::SMatrix{3, 4, Float64}, stanceparams::StanceParams,  gaitparams::GaitParams, mvref::MovementReference)
#     zmeas = sum(stancefootlocations[3, :]) / length(stancefootlocations[3, :])
#     (Δp, ΔR) = skiincrement(zmeas, stanceparams, mvref, gaitparams)
#     incrementedlocations = ΔR * stancefootlocations .+ Δp
#     return incrementedlocations
# end


def stance_foot_locations(
    stance_foot_locations, stance_params, gait_params, movement_reference
):
    """Find the next desired locations for all feet.
    
    Parameters
    ----------
    stance_foot_locations : [type]
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

    z_measured = sum(stance_foot_locations[2, :]) / len(stance_foot_locations[2, :])
    (delta_p, delta_R) = ski_increment(
        z_measured, stance_params, movement_reference, gait_params
    )
    incremented_locations = delta_R @ stance_foot_locations + delta_p
    return incremented_locations


# function stancefootlocation(stancefootlocation::SVector{3, Float64}, stanceparams::StanceParams, gaitparams::GaitParams, mvref::MovementReference)
#     #=
#     Return the incremented location for a specific foot in stance.
#     =#
#     zmeas = stancefootlocation[3]
#     (Δp, ΔR) = skiincrement(zmeas, stanceparams, mvref, gaitparams)
#     incrementedlocation = ΔR * stancefootlocation .+ Δp
#     return incrementedlocation
# end


def stance_foot_location(
    stance_foot_location, stance_params, gait_params, movement_reference
):
    """Find the next desired location for a foot in stance.
    
    Parameters
    ----------
    stance_foot_location : [type]
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
    z_measured = stance_foot_location[2]
    (delta_p, delta_R) = ski_increment(
        z_measured, stance_params, movement_reference, gait_params
    )
    incremented_location = delta_R @ stance_foot_location + delta_p
    return incremented_location
