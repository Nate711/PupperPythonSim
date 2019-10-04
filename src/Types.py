import numpy as np
from scipy.linalg import solve


class MovementCommand:
    def __init__(self):
        self.v_xy_ref = np.array([0, 0])
        self.wz_ref = 0.0
        self.z_ref = -0.16


class MovementReference:
    def __init__(self):
        self.v_xy_ref = np.array([0, 0])
        self.wz_ref = 0.0
        self.z_ref = -0.16


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
    
    @property
    def z_clearance(self):
        return self.__z_clearance
    
    @z_clearance.setter
    def z_clearance(self, z):
        self.__z_clearance = z
        b_z = np.array([0, 0, 0, 0, self.__z_clearance])
        A_z = np.array(
            [
                [0, 0, 0, 0, 1],
                [1, 1, 1, 1, 1],
                [0, 0, 0, 1, 0],
                [4, 3, 2, 1, 0],
                [0.5 ** 4, 0.5 ** 3, 0.5 ** 2, 0.5 ** 1, 0.5 ** 0],
            ]
        )
        self.z_coeffs = solve(A_z, b_z)

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
