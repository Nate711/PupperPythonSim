import pybullet as p
import pybullet_data
import time

from src import PupperXMLParser
from src.Controller import Controller, step_controller
from src.PupperConfig import PupperConfig, EnvironmentConfig, SolverConfig, SwingParams
import numpy as np


def parallel_to_serial_joint_angles(joint_matrix):
    """Convert from joint angles meant for the parallel linkage in 
    Pupper to the joint angles in the serial linkage approximation implemented in the simulation
    
    Parameters
    ----------
    joint_matrix : Numpy array (3, 4)
        Joint angles for parallel linkage
    
    Returns
    -------
    Numpy array (3, 4)
        Joint angles for equivalent serial linkage
    """
    temp = joint_matrix
    temp[2, :] -= joint_matrix[1, :]
    return temp


# Set up PyBullet Simulator
physicsClient = p.connect(p.GUI)  # or p.DIRECT for non-graphical version
p.setAdditionalSearchPath(pybullet_data.getDataPath())  # optionally
p.setGravity(0, 0, -9.81)
pupperId = p.loadMJCF("src/pupper_out.xml")
print("")
print("Pupper bodies IDs:", pupperId)
numjoints = p.getNumJoints(pupperId[1])
print("Number of joints in converted MJCF: ", numjoints)
print("Joint Info: ")
for i in range(numjoints):
    print(p.getJointInfo(pupperId[1], i))
joint_indices = list(range(0, 24, 2))

# Create environment objects
PUPPER_CONFIG = PupperConfig()
ENVIRONMENT_CONFIG = EnvironmentConfig()
SOLVER_CONFIG = SolverConfig()

# Initailize MuJoCo
PupperXMLParser.Parse(PUPPER_CONFIG, ENVIRONMENT_CONFIG, SOLVER_CONFIG)

# Create pupper_controller
pupper_controller = Controller()
pupper_controller.movement_reference.v_xy_ref = np.array([0.0, 0.0])
pupper_controller.movement_reference.wz_ref = 0.0
pupper_controller.swing_params.z_clearance = 0.03
pupper_controller.gait_params.dt = 0.005
pupper_controller.stance_params.delta_y = 0.09

# Run the simulation
timesteps = 60000

# Calculate how many simulation steps to make for every controller update
pupper_update_rate = int(
    1.0 / pupper_controller.gait_params.dt
)  # control rate, updates per second
sim_rate = int(1 / ENVIRONMENT_CONFIG.DT)  # simulation updates per second
sim_steps_per_control_step = int(sim_rate / pupper_update_rate)

for i in range(timesteps):
    # Step the pupper controller forward
    if i % sim_steps_per_control_step == 0:
        # step_controller takes between 0.3ms and 1ms to complete! Definitely fast enough!
        step_controller(pupper_controller)
        serial_joint_angles = parallel_to_serial_joint_angles(pupper_controller.joint_angles)

        p.setJointMotorControlArray(
            bodyUniqueId=pupperId[1],
            jointIndices=joint_indices,
            controlMode=p.POSITION_CONTROL,
            targetPositions=list(serial_joint_angles.T.reshape(12)),
         # positionGains=[1]*12,
         # velocityGains=[1]*12,
            forces=[2]*12,
        )

    p.stepSimulation()
    time.sleep(ENVIRONMENT_CONFIG.DT)