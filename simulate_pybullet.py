import pybullet as p
import pybullet_data
import time
import numpy as np

from src import PupperXMLParser
from src.Controller import step_controller, Controller
from src.HardwareInterface import send_servo_commands, initialize_pwm
from src.PupperConfig import (
    PupperConfig,
    MovementReference,
    GaitParams,
    StanceParams,
    SwingParams,
    ServoParams,
    PWMParams,
    EnvironmentConfig,
    SolverConfig
)
from src.UserInput import UserInputs, get_input, update_controller


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
pupperId = p.loadMJCF("src/pupper_pybullet_out.xml")

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
PUPPER_CONFIG.XML_IN = "pupper_pybullet.xml"
PUPPER_CONFIG.XML_OUT = "pupper_pybullet_out.xml"


ENVIRONMENT_CONFIG = EnvironmentConfig()
SOLVER_CONFIG = SolverConfig()

# Initailize MuJoCo
PupperXMLParser.Parse(PUPPER_CONFIG, ENVIRONMENT_CONFIG, SOLVER_CONFIG)

# Create controller
robot_config = PupperConfig()
servo_params = ServoParams()
controller = Controller(robot_config)
user_input = UserInputs()

# Run the simulation
timesteps = 240*60*10 # simulate for a max of 10 minutes

# Sim seconds per sim step
sim_steps_per_sim_second = 240
sim_seconds_per_sim_step = 1.0 / sim_steps_per_sim_second

start = time.time()
last_control_update = 0

# controller.gait_params.contact_phases = np.array(
#     [[1, 1, 1, 0], [1, 0, 1, 1], [1, 0, 1, 1], [1, 1, 1, 0]]
# )
controller.swing_params.z_clearance = 0.03
controller.movement_reference.v_xy_ref = np.array([0.10, 0.0])
controller.movement_reference.wz_ref = 0.5

# To account for the fact that the CoM of the robot is a little behind the geometric center, 
# put the robot feet a little behind the geometric center to try to match the actual CoM
controller.stance_params.x_shift = -0.01

(hey, now) = (0, 0)

for steps in range(timesteps):
    current_time = time.time()

    # Simulated time can be computed as sim_seconds_per_sim_step * steps
    simluated_time_elapsed = sim_seconds_per_sim_step * steps

    if simluated_time_elapsed - last_control_update > controller.gait_params.dt:
        # This block usually takes < 1ms to run, but every 10 or so iterations it takes as many as 50ms to run
        
        hey = time.time()
        last_control_update = simluated_time_elapsed

        (pos, q_scalar_last) = p.getBasePositionAndOrientation(1)
        q_corrected = (q_scalar_last[3], q_scalar_last[0], q_scalar_last[1], q_scalar_last[2])
        print("Orientation: ", q_corrected)

        # Calculate the next joint angle commands
        step_controller(controller, robot_config, q_corrected)

        # Convert the joint angles from the parallel linkage to the simulated serial linkage
        serial_joint_angles = parallel_to_serial_joint_angles(
            controller.joint_angles
        )

        # Send the joint angles to the sim
        p.setJointMotorControlArray(
            bodyUniqueId=pupperId[1],
            jointIndices=joint_indices,
            controlMode=p.POSITION_CONTROL,
            targetPositions=list(serial_joint_angles.T.reshape(12)),
            # positionGains=[1]*12,
            # velocityGains=[1]*12,
            forces=[4] * 12,
        )

        now = time.time()

    # Simulate physics for 1/240 seconds (1/240 is the default timestep)
    p.stepSimulation()

    time.sleep(0.01)

    # Performance testing
    elapsed = time.time() - start
    if (steps % 60) == 0:
        print(
            "Sim seconds elapsed: {}, Real seconds elapsed: {}".format(
                round(simluated_time_elapsed,3), round(elapsed,3)
            )
        )
        # print("Average steps per second: {0}, elapsed: {1}, i:{2}".format(steps / elapsed, elapsed, i))
