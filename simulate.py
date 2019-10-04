from mujoco_py import load_model_from_path, MjSim, MjViewer, functions
from src import PupperXMLParser
from src.Controller import Controller, step_controller
import numpy as np
from src.PupperConfig import PupperConfig, EnvironmentConfig, SolverConfig

# Create environment objects
PUPPER_CONFIG = PupperConfig()
ENVIRONMENT_CONFIG = EnvironmentConfig()
SOLVER_CONFIG = SolverConfig()

# Initailize MuJoCo
PupperXMLParser.Parse(PUPPER_CONFIG, ENVIRONMENT_CONFIG, SOLVER_CONFIG)
model = load_model_from_path("pupper_out.xml")
sim = MjSim(model)
viewer = MjViewer(sim)

# Create pupper_controller
pupper_controller = Controller()

# Run the simulation
timesteps = ENVIRONMENT_CONFIG.SIM_STEPS

pupper_update_rate = PUPPER_CONFIG.UPDATE_RATE  # control rate, steps per second
sim_rate = int(1 / ENVIRONMENT_CONFIG.DT)  # simulation steps per second
sim_steps_per_control_step = int(sim_rate / pupper_update_rate)

for i in range(timesteps):
    # Step the pupper controller forward
    step_controller(pupper_controller)
    if i % sim_steps_per_control_step == 0:
        sim.data.ctrl[:] = pupper_controller.joint_angles

    sim.step()
    viewer.render()