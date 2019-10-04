import numpy as np


class PupperConfig:
    def __init__(self):
        # XML files
        self.XML_IN = "pupper.xml"
        self.XML_OUT = "pupper_out.xml"

        # Robot geometry
        self.LEG_FB = 0.10  # front-back distance from center line to leg axis
        self.LEG_LR = 0.0419  # left-right distance from center line to leg plane
        self.LEG_L = 0.125
        self.LEG_L2 = 0.125
        self.LEG_L1 = 0.1235
        self.ABDUCTION_OFFSET = 0.027  # distance from abduction axis to leg
        self.FOOT_RADIUS = 0.01

        self.HIP_L = 0.0394
        self.HIP_W = 0.0744
        self.HIP_T = 0.0214
        self.HIP_OFFSET = 0.0132

        self.L = 0.276
        self.W = 0.100
        self.T = 0.050

        self.LEG_ORIGINS = np.array(
            [
                [self.LEG_FB, self.LEG_FB, -self.LEG_FB, -self.LEG_FB],
                [-self.LEG_LR, self.LEG_LR, -self.LEG_LR, self.LEG_LR],
                [0, 0, 0, 0],
            ]
        )

        self.ABDUCTION_OFFSETS = np.array(
            [
                -self.ABDUCTION_OFFSET,
                self.ABDUCTION_OFFSET,
                -self.ABDUCTION_OFFSET,
                self.ABDUCTION_OFFSET,
            ]
        )

        self.START_HEIGHT = 0.2

        # Robot inertia params
        self.FRAME_MASS = 0.560  # kg
        self.MODULE_MASS = 0.080  # kg
        self.LEG_MASS = 0.030  # kg
        self.MASS = self.FRAME_MASS + (self.MODULE_MASS + self.LEG_MASS) * 4

        # Compensation factor of 2 because the inertia measurement was just
        # of the carbon fiber and plastic parts of the frame and did not
        # include the hip servos and electronics
        self.FRAME_INERTIA = tuple(
            map(lambda x: 2.0 * x, (1.844e-4, 1.254e-3, 1.337e-3))
        )
        self.MODULE_INERTIA = (3.698e-5, 7.127e-6, 4.075e-5)
        self.LEG_INERTIA = (2.253e-4, 6.493e-5, 2.502e-4)

        # Joint params
        G = 220  # Servo gear ratio
        m_rotor = 0.016  # Servo rotor mass
        r_rotor = 0.005  # Rotor radius
        self.ARMATURE = G ** 2 * m_rotor * r_rotor ** 2  # Inertia of rotational joints
        # print("Servo armature", self.ARMATURE)

        NATURAL_DAMPING = 1.0  # Damping resulting from friction
        ELECTRICAL_DAMPING = 0.049  # Damping resulting from back-EMF
        
        self.REV_DAMPING = (
            NATURAL_DAMPING + ELECTRICAL_DAMPING
        )  # Damping torque on the revolute joints

        # Servo params
        self.SERVO_REV_KP = 300  # Position gain [Nm/rad]

        # Force limits
        self.MAX_JOINT_TORQUE = 3.0
        self.REVOLUTE_RANGE = 1.57


class EnvironmentConfig:
    def __init__(self):
        self.MU = 1.5  # coeff friction
        self.DT = 0.001  # seconds between simulation steps


class SolverConfig:
    def __init__(self):
        self.JOINT_SOLREF = "0.001 1"  # time constant and damping ratio for joints
        self.JOINT_SOLIMP = "0.9 0.95 0.001"  # joint constraint parameters
        self.GEOM_SOLREF = (
            "0.005 2"
        )  # time constant and damping ratio for geom contacts
        self.GEOM_SOLIMP = "0.9 0.95 0.001"  # geometry contact parameters
