# include("StanceController.jl")
# include("Gait.jl")
# include("Types.jl")

from Types import *
import numpy as np
from transforms3d.euler import euler2mat


# function raibert_tdlocations(swingparams::SwingParams, stanceparams::StanceParams, gaitparams::GaitParams, mvref::MovementReference)
# 	#=
# 	Use the raibert heuristic to find the touchdown locations for all legs as if they were all in swing
# 	=#

# 	# Contruct the 'skis' defined by p and R
# 	p2 = swingparams.alpha * gaitparams.stanceticks * gaitparams.dt * mvref.vxyref
# 	p = SVector(p2[1], p2[2], 0.0)
# 	Θ = swingparams.beta * gaitparams.stanceticks * gaitparams.dt * mvref.wzref
# 	R = RotZ(Θ)
# 	tdlocations = R * SMatrix(stanceparams.defaultstance) .+ p
# 	return tdlocations
# end


def raibert_touchdown_locations(
    swing_params, stance_params, gait_params, movement_reference
):
    """[summary]
    
    Parameters
    ----------
    swing_params : [type]
        [description]
    stance_params : [type]
        [description]
    gait_params : [type]
        [description]
    movement_reference : [type]
        [description]
    """
    p_temp = (
        swing_params.alpha
        * gait_params.stance_ticks
        * gait_params.dt
        * movement_reference.v_xy_ref
    )
    p = np.array([p_temp[0], p_temp[1], 0.0])
    theta = (
        swing_params.beta
        * gait_params.stance_ticks
        * gait_params.dt
        * movement_reference.wz_ref
    )
    R = euler2mat(0, 0, theta)
    return R * stance_params.default_stance + p


# function raibert_tdlocation(legindex::Integer, swingparams::SwingParams, stanceparams::StanceParams, gaitparams::GaitParams, mvref::MovementReference)
# 	#=
# 	Use the raibert heuristic to find the touchdown location for a specific leg in swing.
# 	=#
# 	out::SVector{3, Float64} = raibert_tdlocations(swingparams, stanceparams, gaitparams, mvref)[:, legindex]
# 	return out
# end


def raibert_touchdown_location(
    leg_index, swing_params, stance_params, gait_params, movement_reference
):
    """[summary]
    
    Parameters
    ----------
    leg_index : [type]
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
    p_temp = (
        swing_params.alpha
        * gait_params.stance_ticks
        * gait_params.dt
        * movement_reference.v_xy_ref
    )
    p = np.array([p_temp[0], p_temp[1], 0.0])
    theta = (
        swing_params.beta
        * gait_params.stance_ticks
        * gait_params.dt
        * movement_reference.wz_ref
    )
    R = euler2mat(0, 0, theta)
    return R @ stance_params.default_stance[:, leg_index] + p


# function swingheight(swingphase::Number, swingparams::SwingParams; triangular=true)
# 	if triangular
# 		if swingphase < 0.5
# 			swingheight_ = swingphase / 0.5 * swingparams.zclearance
# 		else
# 			swingheight_ = swingparams.zclearance * (1 - (swingphase - 0.5) / 0.5)
# 		end
# 	else
# 		timevec = SVector{5}(swingphase^4, swingphase^3, swingphase^2, swingphase, 1)
# 		swingheight_ = timevec' * swingparams.zcoeffs # vertical offset relative to the stance default
# 	end
# 	return swingheight_
# end


def swing_height(swing_phase, swing_params, triangular=True):
    """[summary]
    
    Parameters
    ----------
    swing_phase : [type]
        [description]
    swing_params : [type]
        [description]
    triangular : bool, optional
        [description], by default True
    """
    if triangular:
        if swing_phase < 0.5:
            swing_height_ = swing_phase / 0.5 * swing_params.z_clearance
        else:
            swing_height_ = swing_params.z_clearance * (1 - (swing_phase - 0.5) / 0.5)
    else:
        time_vec = np.array(
            [swing_phase ** 4, swing_phase ** 3, swing_phase ** 2, swing_phase, 1]
        )
        swing_height_ = np.dot(time_vec, swing_params.z_coeffs)
    return swing_height_


# function swingfootlocations(swingprop::Number, footlocations::SMatrix{3, 4, Float64}, swingparams::SwingParams, stanceparams::StanceParams, gaitparams::GaitParams, mvref::MovementReference)
# 	#=
# 	Return the incremental position update for the legs as if they were all in swing

# 	swingprop: Proportion of swing phase completed.
# 	footlocations: Current foot locations
# 	=#

# 	@assert swingprop >= 0 && swingprop <= 1.0

# 	# swingheight is the height where the swing feet should be at the *given time*,
# 	# while tdlocations is the location that the feet should be at at the *end* of the swing period.
# 	swingheight_ = swingheight(swingprop, swingparams)
# 	tdlocations = raibert_tdlocations(swingparams, stanceparams, gaitparams, mvref)
# 	timeleft = gaitparams.dt * gaitparams.swingticks * (1 - swingprop)
# 	v = (tdlocations - footlocations) / timeleft .* SVector{3}(1, 1, 0)
# 	Δfootlocations = v * gaitparams.dt
# 	swingheight_matrix = SMatrix{3, 4, Float64}(0, 0, swingheight_ + mvref.zref,
# 											0, 0, swingheight_ + mvref.zref,
# 											0, 0, swingheight_ + mvref.zref,
# 											0, 0, swingheight_ + mvref.zref)
# 	incrementedlocations = swingheight_matrix + Δfootlocations
# 	return incrementedlocations
# end

# function swingfootlocation(swingprop::Number, footlocation::SVector{3, Float64}, legindex::Integer, swingparams::SwingParams, stanceparams::StanceParams, gaitparams::GaitParams, mvref::MovementReference)
# 	#=
# 	Return the new location for a specific foot in swing.

# 	swingprop: Proportion of swing phase completed.
# 	footlocations: Current foot locations
# 	=#

# 	@assert swingprop >= 0 && swingprop <= 1.0

# 	# swingheight is the height where the swing feet should be at the *given time*,
# 	# while tdlocations is the location that the feet should be at at the *end* of the swing period.
# 	swingheight_ = swingheight(swingprop, swingparams)
# 	tdlocation = raibert_tdlocation(legindex, swingparams, stanceparams, gaitparams, mvref)
# 	timeleft = gaitparams.dt * gaitparams.swingticks * (1.0 - swingprop)
# 	v = (tdlocation - footlocation) / timeleft .* SVector{3}(1, 1, 0)
# 	Δfootlocation = v * gaitparams.dt
# 	zvector = SVector{3, Float64}(0, 0, swingheight_ + mvref.zref)
# 	incrementedlocation::SVector{3, Float64} = footlocation .* SVector{3}(1, 1, 0) + zvector + Δfootlocation
# 	return incrementedlocation
# end


def swing_foot_location(
    swing_prop,
    foot_location,
    leg_index,
    swing_params,
    stance_params,
    gait_params,
    movement_reference,
):
    """[summary]
    
    Parameters
    ----------
    swing_prop : [type]
        [description]
    foot_location : [type]
        [description]
    leg_index : [type]
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
    assert swing_prop >= 0 and swing_prop <= 1

    swing_height_ = swing_height(swing_prop, swing_params)
    touchdown_location = raibert_touchdown_location(
        leg_index, swing_params, stance_params, gait_params, movement_reference
    )
    time_left = gait_params.dt * gait_params.swing_ticks * (1.0 - swing_prop)
    v = (touchdown_location - foot_location) / time_left * np.array([1, 1, 0])
    delta_foot_location = v * gait_params.dt
    z_vector = np.array([0, 0, swing_height_ + movement_reference.z_ref])
    return foot_location * np.array([1, 1, 0]) + z_vector + delta_foot_location
