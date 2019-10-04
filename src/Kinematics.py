# using Parameters
# using Rotations
# include("PupperConfig.jl")

import numpy as np
from transforms3d.euler import euler2mat

# function assertValidLeg(i::Integer)
# 	@assert i >= 1 && i <= 4
# 	return nothing
# end


def _assert_valid_leg(i):
    """Assert that the given leg index is valid
    
    Parameters
    ----------
    i : int
        Leg index.
    """
    assert i in [0, 1, 2, 3]


# function legForwardKinematics!(r_body::Vector{Float64}, α::Vector{Float64}, i::Integer, config::PupperConfig)
# 	assertValidLeg(i)

# 	beta = α[1]
# 	theta = α[2]
# 	r = α[3]
# 	if i == 2 || i == 4
# 		unrotated = [0, config.ABDUCTION_OFFSET::Float64, -config.LEG_L::Float64 + r]
# 	else
# 		unrotated = [0, -config.ABDUCTION_OFFSET::Float64, -config.LEG_L::Float64 + r]
# 	end

# 	# RotXY is equiv to an intrinsic rotation first around the x axis, and then the new y axis
# 	r_body .= RotXY(beta, theta) * unrotated
# end

# function legForwardKinematics(α::SVector{3, Float64}, i::Integer, config::PupperConfig)
# 	#=
# 	Given the joint angles, return the vector from the hip to the foot in the body frame.

# 	α: Joint angles.
# 	i: Leg number. 1 = front right, 2 = front left, 3 = back right, 4 = back left.

# 	return: Vector from hip to foot in body coordinates.
# 	=#
# 	assertValidLeg(i)

# 	beta = α[1]
# 	theta = α[2]
# 	r = α[3]
# 	y = config.ABDUCTION_OFFSETS[i]
# 	unrotated_leg = SVector(0.0, y, -config.LEG_L + r)

# 	# RotXY is equiv to an intrinsic rotation first around the x axis, and then the new y axis
# 	return RotXY(beta, theta) * unrotated_leg
# end


def leg_forward_kinematics(alpha, leg_index, config):
    """Find the body-centric coordinates of a given foot given the joint angles.
    
    Parameters
    ----------
    alpha : Numpy array (3)
        Joint angles ordered as (abduction, hip, knee)
    leg_index : int
        Leg index.
    config : Config object
        Robot parameters object
    
    Returns
    -------
    Numpy array (3)
        Body-centric coordinates of the specified foot
    """
    _assert_valid_leg(leg_index)
    y = config.ABDUCTION_OFFSET[leg_index]
    unrotated_leg = np.array([0, y, -config.LEG_L + alpha[2]])
    return euler2mat(alpha[0], alpha[1], "sxyz") * unrotated_leg


# function leg_explicitinversekinematics(r_body_foot::SVector{3, Float64}, i::Integer, config::PupperConfig)
# 	assertValidLeg(i)

# 	# Unpack vector from body to foot
# 	x, y, z = r_body_foot

# 	# Distance from the leg origin to the foot, projected into the y-z plane
# 	R_body_foot_yz = (y ^ 2 + z ^ 2) ^ 0.5

# 	# Distance from the leg's forward/back point of rotation to the foot
# 	R_hip_foot_yz = (R_body_foot_yz ^ 2 - config.ABDUCTION_OFFSET ^ 2) ^ 0.5

# 	# Ensure that the target point is reachable
# 	@assert R_body_foot_yz >= abs(config.ABDUCTION_OFFSET)

# 	# Interior angle of the right triangle formed in the y-z plane by the leg that is coincident to the ab/adduction axis
# 	# For feet 2 (front left) and 4 (back left), the abduction offset is positive, for the right feet, the abduction offset is negative.
# 	ϕ = acos(config.ABDUCTION_OFFSETS[i] / R_body_foot_yz)

# 	# Angle of the y-z projection of the hip-to-foot vector, relative to the positive y-axis
# 	θ_ = atan(z, y)

# 	# Ab/adduction angle, relative to the positive y-axis
# 	α1 = ϕ + θ_

# 	# θ: Angle between the tilted negative z-axis and the hip-to-foot vector
# 	θ = atan(-x, R_hip_foot_yz)

# 	# Distance between the hip and foot
# 	R_hip_foot = (R_hip_foot_yz ^ 2 + x ^ 2) ^ 0.5

# 	# Angle between the line going from hip to foot and the link L1
# 	ψ = acos((config.LEG_L1 ^ 2 + R_hip_foot ^ 2 - config.LEG_L2 ^ 2) / (2 * config.LEG_L1 * R_hip_foot))

# 	# Angle of the first link relative to the tilted negative z axis
# 	α2 = θ + ψ

# 	# Angle between the leg links L1 and L2
# 	β = acos((config.LEG_L1 ^ 2 + config.LEG_L2 ^ 2 - R_hip_foot ^ 2) / (2 * config.LEG_L1 * config.LEG_L2))

# 	# Angle of the second link relative to the tilted negative z axis
# 	α3 = α2 - (π - β)
# 	return SVector(α1, α2, α3)
# end


def leg_explicit_inverse_kinematics(r_body_foot, leg_index, config):
    """Find the joint angles corresponding to the given body-relative foot position for a given leg and configuration
    
    Parameters
    ----------
    r_body_foot : [type]
        [description]
    leg_index : [type]
        [description]
    config : [type]
        [description]
    
    Returns
    -------
    numpy array (3)
        Array of corresponding joint angles.
    """
    _assert_valid_leg(leg_index)
    (x, y, z) = r_body_foot

    # Distance from the leg origin to the foot, projected into the y-z plane
    R_body_foot_yz = (y ** 2 + z ** 2) ** 0.5

    # Distance from the leg's forward/back point of rotation to the foot
    R_hip_foot_yz = (R_body_foot_yz ** 2 - config.ABDUCTION_OFFSET ** 2) ** 0.5

    # Ensure that the target point is reachable
    assert R_body_foot_yz >= abs(config.ABDUCTION_OFFSET)

    # Interior angle of the right triangle formed in the y-z plane by the leg that is coincident to the ab/adduction axis
    # For feet 2 (front left) and 4 (back left), the abduction offset is positive, for the right feet, the abduction offset is negative.
    phi = np.arccos(config.ABDUCTION_OFFSETS[leg_index] / R_body_foot_yz)

    # Angle of the y-z projection of the hip-to-foot vector, relative to the positive y-axis
    hip_foot_angle = np.arctan2(z, y)

    # Ab/adduction angle, relative to the positive y-axis
    abduction_angle = phi + hip_foot_angle

    # theta: Angle between the tilted negative z-axis and the hip-to-foot vector
    theta = np.arctan2(-x, R_hip_foot_yz)

    # Distance between the hip and foot
    R_hip_foot = (R_hip_foot_yz ** 2 + x ** 2) ** 0.5

    # Angle between the line going from hip to foot and the link L1
    trident = np.arccos(
        (config.LEG_L1 ** 2 + R_hip_foot ** 2 - config.LEG_L2 ** 2)
        / (2 * config.LEG_L1 * R_hip_foot)
    )

    # Angle of the first link relative to the tilted negative z axis
    hip_angle = theta + trident

    print(R_body_foot_yz, R_hip_foot_yz, config.LEG_L1, config.LEG_L2, R_hip_foot)
    # Angle between the leg links L1 and L2
    beta = np.arccos(
        (config.LEG_L1 ** 2 + config.LEG_L2 ** 2 - R_hip_foot ** 2)
        / (2 * config.LEG_L1 * config.LEG_L2)
    )

    # Angle of the second link relative to the tilted negative z axis
    knee_angle = hip_angle - (np.pi - beta)

    return np.array([abduction_angle, hip_angle, knee_angle])


# function fourlegs_inversekinematics(r_body_foot::SMatrix{3, 4, Float64}, config::PupperConfig)
# 	#=
# 	Compute the joint angles for all four robot legs.

# 	Allocation-free.
# 	=#
# 	α = zeros(SMatrix{3, 4, Float64})
# 	for i in 1:4
# 		body_offset = config.LEG_ORIGINS[:, i]
# 		temp::SVector{3, Float64} = leg_explicitinversekinematics(r_body_foot[:, i] - body_offset, i, config)
# 		for j in 1:3
# 			α = setindex(α, temp[j], LinearIndices(α)[j, i])
# 		end
# 	end
# 	return α
# end


def four_legs_inverse_kinematics(r_body_foot, config):
    """Find the joint angles for all twelve DOF correspoinding to the given matrix of body-relative foot positions.
    
    Parameters
    ----------
    r_body_foot : numpy array (3,4)
        Matrix of the body-frame foot positions. Each column corresponds to a separate foot.
    config : Config object
        Object of robot configuration parameters.
    
    Returns
    -------
    numpy array (3,4)
        Matrix of corresponding joint angles.
    """
    alpha = np.zeros((3, 4))
    for i in range(4):
        body_offset = config.LEG_ORIGINS[:, i]
        alpha[:, i] = leg_explicit_inverse_kinematics(
            r_body_foot[:, i] - body_offset, i, config
        )
    return alpha
