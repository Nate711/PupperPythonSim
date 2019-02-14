"""
Per-robot configuration file that is particular to each individual robot, not just the type of robot.
"""
import numpy as np


MICROS_PER_RAD = 11.333 * 180.0 / np.pi  # Must be calibrated
NEUTRAL_ANGLE_DEGREES = np.array(
    [[0, 4, 4, -11], [44, 39, 38, 46], [-21, -26, -35, -49]]
)

