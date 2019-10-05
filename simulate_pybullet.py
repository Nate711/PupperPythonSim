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
pupper_controller.movement_reference.v_xy_ref = np.array([0.2, 0.0])
pupper_controller.movement_reference.wz_ref = 0.0
pupper_controller.swing_params.z_clearance = 0.03  # Changing to be higher
pupper_controller.gait_params.dt = 0.01  # Simulated seconds per controller step
# Whole sim  is  set to run about 600ms per gate
pupper_controller.stance_params.delta_y = 0.1

# Run the simulation
timesteps = 60000

# Sim seconds per sim step
sim_steps_per_sim_second = 1000
sim_seconds_per_sim_step = 1.0 / sim_steps_per_sim_second
p.setTimeStep(1.0 / sim_steps_per_sim_second)


start = time.time()
last_control_update = 0
for steps in range(timesteps):
    # Step the pupper controller forward
    current_time = time.time()

    # Simulated time can be computed as sim_seconds_per_sim_step * steps
    simluated_time_elapsed = sim_seconds_per_sim_step * steps

    # Want a function that start at 100 then linearly ramps to 0 over 1 second then stays at 0
    p.setGravity(0, 0, -9.81 - max(0, (20 - simluated_time_elapsed * 10)))

    if simluated_time_elapsed - last_control_update > pupper_controller.gait_params.dt:
        last_control_update = simluated_time_elapsed
        # step_controller takes between 0.3ms and 1ms to complete! Definitely fast enough!

        # This will move the joints far enough to last gait_params.dt seconds
        # If we want the legs to move the correct distance in simulated time, we need to tell the
        # Robot how many *simulated* seconds have ellapse
        step_controller(pupper_controller)

        serial_joint_angles = parallel_to_serial_joint_angles(
            pupper_controller.joint_angles
        )
        # t2 = time.time()
        p.setJointMotorControlArray(
            bodyUniqueId=pupperId[1],
            jointIndices=joint_indices,
            controlMode=p.POSITION_CONTROL,
            targetPositions=list(serial_joint_angles.T.reshape(12)),
            # positionGains=[1]*12,
            # velocityGains=[1]*12,
            forces=[2] * 12,
        )
        # print(t2-now, ",", time.time()-t2)

    p.stepSimulation()
    # time.sleep(ENVIRONMENT_CONFIG.DT)

    # Perf testing
    elapsed = time.time() - start
    if (steps % 100) == 0:
        print(
            "Sim seconds elapsed: {}, Real seconds elapsed: {}".format(
                simluated_time_elapsed, elapsed
            )
        )
        # print("Average steps per second: {0}, elapsed: {1}, i:{2}".format(i / elapsed, elapsed, i))
