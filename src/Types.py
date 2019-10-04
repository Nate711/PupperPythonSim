import numpy as np
from scipy.linalg import solve

# @with_kw mutable struct MovementCommand
#     vxydes::MVector{2, Float64} = MVector(0, 0)
#     wzdes::Float64 = 0
#     zdes::Float64 = -0.18
# end


class MovementCommand:
    def __init__(self):
        self.v_xy_ref = np.array([0, 0])
        self.wz_ref = 0.0
        self.z_ref = -0.16


# Note: Mutable structs are allocated on the heap
# @with_kw struct MovementReference
#     vxyref::SVector{2, Float64} = SVector(0, 0)
#     wzref::Float64 = 0
#     zref::Float64 = -0.125
# end


class MovementReference:
    def __init__(self):
        self.v_xy_ref = np.array([0, 0])
        self.wz_ref = 0.0
        self.z_ref = -0.16


# Note: Using an MMatrix for the defaultstance or making the struct mutable results in 180k more allocatinons
# @with_kw struct StanceParams
#     # Time constant in [sec] for the feet in stance to move towards the reference z height
#     ztimeconstant::Float64 = 1.0
#     Δx::Float64 = 0.1
#     Δy::Float64 = 0.09
#     defaultstance::SMatrix{3, 4, Float64, 12} = SMatrix{3,4, Float64}(	Δx, -Δy, 0,
#                                                                    		Δx,  Δy, 0,
#                                                                     	-Δx, -Δy, 0,
#                                                                     	-Δx, Δy, 0)
# end
class StanceParams:
    def __init__(self):
        self.z_time_constant = 1.0
        self.delta_x = 0.1
        self.delta_y = 0.09
        self.default_stance = np.array(
            [
                [self.delta_x, self.delta_x, -self.delta_x, -self.delta_x],
                [-self.delta_y, self.delta_y, -self.delta_y, self.delta_y],
                [0, 0, 0, 0],
            ]
        )


# @with_kw struct SwingParams
# 	zclearance::Float64 = 0.01
# 	Az::SMatrix{5, 5, Float64} = SMatrix{5,5}(	0, 1, 0, 4, 0.5^4,
# 												0, 1, 0, 3, 0.5^3,
# 												0, 1, 0, 2, 0.5^2,
# 												0, 1, 1, 1, 0.5^1,
# 												1, 1, 0, 0, 0.5^0)
# 	bz = SVector{5, Float64}(0, 0, 0, 0, zclearance)
# 	zcoeffs::SVector{5, Float64} = Az \ bz
#     alpha::Float64 = 0.5 # Ratio between touchdown distance and total horizontal stance movement
# 	beta::Float64 = 0.5 # Ratio between touchdown distance and total horizontal stance movement
# end


class SwingParams:
    def __init__(self):
        self.z_clearance = 0.01
        A_z = np.array(
            [
                [0, 0, 0, 0, 1],
                [1, 1, 1, 1, 1],
                [0, 0, 0, 1, 0],
                [4, 3, 2, 1, 0],
                [0.5 ** 4, 0.5 ** 3, 0.5 ** 2, 0.5 ** 1, 0.5 ** 0],
            ]
        )
        b_z = np.array([0, 0, 0, 0, self.z_clearance])
        self.z_coeffs = solve(A_z, b_z)
        self.alpha = (
            0.5
        )  # Ratio between touchdown distance and total horizontal stance movement
        self.beta = (
            0.5
        )  # Ratio between touchdown distance and total horizontal stance movement


# @with_kw struct GaitParams
# 	# Default values are for a trotting gait
# 	dt::Float64 = 0.01

# 	# There are four distinct phases for a trot
# 	numphases::Int = 4

# 	# 4 x num_phase matrix of contacts for each gait phase
# 	contactphases::SMatrix{4, 4, Int, 16} = SMatrix{4, 4, Int, 16}( 1, 1, 1, 1,
# 																	1, 0, 0, 1,
# 																	1, 1, 1, 1,
# 																	0, 1, 1, 0)
# 	overlapticks::Int = round(0.1 / dt)
# 	swingticks::Int = round(0.2 / dt)
# 	stanceticks::Int = 2 * overlapticks + swingticks
# 	phasetimes::SVector{4, Int} = SVector(overlapticks, swingticks, overlapticks, swingticks)
# 	phaselength::Int = 2 * overlapticks + 2 * swingticks
# end


class GaitParams:
    def __init__(self):
        self.dt = 0.01
        self.num_phases = 4
        self.contact_phases = np.array(
            [[1, 1, 1, 0], [1, 0, 1, 1], [1, 0, 1, 1], [1, 1, 1, 0]]
        )
        self.overlap_ticks = int(0.1 / self.dt)
        self.swing_ticks = int(0.2 / self.dt)
        self.stance_ticks = 2 * self.overlap_ticks + self.swing_ticks
        self.phase_times = np.array(
            [self.overlap_ticks, self.swing_ticks, self.overlap_ticks, self.swing_ticks]
        )
        self.phase_length = 2 * self.overlap_ticks + 2 * self.swing_ticks
