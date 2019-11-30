from src import PupperXMLParser
from src.Controller import Controller, step_controller
from src.PupperConfig import (
    PupperConfig,
    EnvironmentConfig,
    SolverConfig,
    SwingParams,
    ServoParams,
    PWMParams,
)
import time
import numpy as np
from mujoco_py import load_model_from_path, MjSim, MjViewer, functions
from src.HardwareInterface import angle_to_pwm


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


# Create environment objects
PUPPER_CONFIG = PupperConfig()
ENVIRONMENT_CONFIG = EnvironmentConfig()
SOLVER_CONFIG = SolverConfig()

# Initailize MuJoCo
PupperXMLParser.Parse(PUPPER_CONFIG, ENVIRONMENT_CONFIG, SOLVER_CONFIG)
model = load_model_from_path("src/pupper_out.xml")
sim = MjSim(model)
viewer = MjViewer(sim)

# Create pupper_controller
pupper_controller = Controller()
pupper_controller.movement_reference.v_xy_ref = np.array([0.2, 0.0])
pupper_controller.movement_reference.wz_ref = 0.0
pupper_controller.swing_params.z_clearance = 0.03
pupper_controller.gait_params.dt = 0.005
pupper_controller.stance_params.delta_y = 0.12

# Initialize joint angles
sim.data.qpos[7:] = parallel_to_serial_joint_angles(
    pupper_controller.joint_angles
).T.reshape(12)
# Set the robot to be above the floor to begin with
sim.data.qpos[2] = 0.5

# Initialize pwm and servo params
pwm_params = PWMParams()
servo_params = ServoParams()

# Run the simulation
timesteps = 60000

# Calculate how many simulation steps to make for every controller update
pupper_update_rate = int(
    1.0 / pupper_controller.gait_params.dt
)  # control rate, updates per second
sim_rate = int(1 / ENVIRONMENT_CONFIG.DT)  # simulation updates per second
sim_steps_per_control_step = int(sim_rate / pupper_update_rate)

for i in range(timesteps):
    # sim.data.qpos[2] = 0.5

    # Step the pupper controller forward
    if i % sim_steps_per_control_step == 0:
        # step_controller takes between 0.3ms and 1ms to complete! Definitely fast enough!
        step_controller(pupper_controller)

        # calculate theoretical pwm values
        pwm_commands = np.zeros((3, 4))
        for axis in range(3):
            for leg in range(4):
                pwm_commands[axis, leg] = angle_to_pwm(
                    pupper_controller.joint_angles[axis, leg], servo_params, axis, leg
                )
        print("Joint Angles: ")
        print(pupper_controller.joint_angles)
        print("Servo Pulse Width (uS): ")
        print(pwm_commands)
        print("")

        # Convert from joint angles meant for a parallel linkage to the serial linkage implemented in Mujoco
        sim.data.ctrl[:] = parallel_to_serial_joint_angles(
            pupper_controller.joint_angles
        ).T.reshape(12)

    sim.step()
    viewer.render()
