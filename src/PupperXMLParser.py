import os


def Parse(PUPPER_CONFIG, ENVIRONMENT_CONFIG, SOLVER_CONFIG):
    """Replace the variable names in the placeholder XML robot file with actual values given from the configuration object parameters.
    
    Parameters
    ----------
    PUPPER_CONFIG : PupperConfig
        Pupper configuration object.
    ENVIRONMENT_CONFIG : EnvironmentConfig
        MuJoCo environment configuration object.
    SOLVER_CONFIG : SolverConfig
        MuJoCo solver configuration object.
    """
    # FILE PATHS
    dir_path = os.path.dirname(os.path.realpath(__file__))
    IN_FILE = os.path.join(dir_path, PUPPER_CONFIG.XML_IN)
    OUT_FILE = os.path.join(dir_path, PUPPER_CONFIG.XML_OUT)

    # ROBOT PARAMETERS

    # Solver params
    pupper_timestep = ENVIRONMENT_CONFIG.DT
    pupper_joint_solref = SOLVER_CONFIG.JOINT_SOLREF
    pupper_joint_solimp = SOLVER_CONFIG.JOINT_SOLIMP
    pupper_geom_solref = SOLVER_CONFIG.GEOM_SOLREF
    pupper_geom_solimp = SOLVER_CONFIG.GEOM_SOLIMP

    # Geometry params
    pupper_leg_radius = PUPPER_CONFIG.FOOT_RADIUS  # radius of leg capsule
    pupper_friction = ENVIRONMENT_CONFIG.MU  # friction between legs and ground
    pupper_half_size = "%s %s %s" % (
        PUPPER_CONFIG.L / 2,
        PUPPER_CONFIG.W / 2,
        PUPPER_CONFIG.T / 2,
    )  # half-size of body box
    # to-from leg geometry
    pupper_leg_geom = "0 0 0 0 0 %s" % (-PUPPER_CONFIG.LEG_L)
    pupper_l1_geom = "0 0 0 0 0 %s" % (-PUPPER_CONFIG.LEG_L1)
    pupper_l2_geom = "0 0 0 0 0 %s" % (-PUPPER_CONFIG.LEG_L2)

    pupper_start_position = "0 0 %s" % (
        PUPPER_CONFIG.START_HEIGHT
    )  # Initial position of the robot torso
    pupper_hip_box = "%s %s %s" % (
        PUPPER_CONFIG.HIP_L / 2,
        PUPPER_CONFIG.HIP_W / 2,
        PUPPER_CONFIG.HIP_T / 2,
    )  # Size of the box representing the hip

    pupper_force_geom = "0 0 -0.34"

    # Mass/Inertia Params
    pupper_armature = PUPPER_CONFIG.ARMATURE  # armature for joints [kgm2]
    pupper_frame_inertia = "%s %s %s" % PUPPER_CONFIG.FRAME_INERTIA
    pupper_module_inertia = "%s %s %s" % PUPPER_CONFIG.MODULE_INERTIA
    pupper_leg_inertia = "%s %s %s" % PUPPER_CONFIG.LEG_INERTIA

    # Joint & servo params
    pupper_rev_kp = PUPPER_CONFIG.SERVO_REV_KP

    pupper_joint_range = "%s %s" % (
        -PUPPER_CONFIG.REVOLUTE_RANGE,
        PUPPER_CONFIG.REVOLUTE_RANGE,
    )  # joint range in rads for angular joints
    pupper_l2_joint_range = "%s %s" % (
        -PUPPER_CONFIG.REVOLUTE_RANGE - PUPPER_CONFIG.REVOLUTE_RANGE,
        PUPPER_CONFIG.REVOLUTE_RANGE - PUPPER_CONFIG.REVOLUTE_RANGE,
    )  # joint range for l2 knee joint
    pupper_rev_torque_range = "%s %s" % (
        -PUPPER_CONFIG.MAX_JOINT_TORQUE,
        PUPPER_CONFIG.MAX_JOINT_TORQUE,
    )  # force range for ab/ad and forward/back angular joints
    pupper_rev_damping = (
        PUPPER_CONFIG.REV_DAMPING
    )  # damping on angular joints [Nm/rad/s]

    # Sensor Noise Parameters #
    pupper_accel_noise = 0.01
    pupper_encoder_noise = 0.001
    pupper_gyro_noise = 0.02
    pupper_encoder_vel_noise = 0.01
    pupper_force_noise = 0

    # Parse the xml
    print("Parsing MuJoCo XML file:")
    print("Input xml: %s" % IN_FILE)
    print("Output xml: %s" % OUT_FILE)

    with open(IN_FILE, "r") as file:
        filedata = file.read()

    # Replace variable names with values

    # Solver specs
    filedata = filedata.replace("pupper_timestep", str(pupper_timestep))
    filedata = filedata.replace("pupper_joint_solref", str(pupper_joint_solref))
    filedata = filedata.replace("pupper_geom_solref", str(pupper_geom_solref))
    filedata = filedata.replace("pupper_friction", str(pupper_friction))
    filedata = filedata.replace("pupper_armature", str(pupper_armature))
    filedata = filedata.replace("pupper_joint_solimp", str(pupper_joint_solimp))
    filedata = filedata.replace("pupper_geom_solimp", str(pupper_geom_solimp))

    # Joint specs
    filedata = filedata.replace("pupper_joint_range", str(pupper_joint_range))
    filedata = filedata.replace("pupper_l2_joint_range", str(pupper_l2_joint_range))
    filedata = filedata.replace("pupper_rev_torque_range", str(pupper_rev_torque_range))
    filedata = filedata.replace("pupper_rev_damping", str(pupper_rev_damping))

    # Servo specs
    filedata = filedata.replace("pupper_rev_kp", str(pupper_rev_kp))

    # Geometry specs
    filedata = filedata.replace("pupper_frame_mass", str(PUPPER_CONFIG.FRAME_MASS))
    filedata = filedata.replace("pupper_module_mass", str(PUPPER_CONFIG.MODULE_MASS))
    filedata = filedata.replace("pupper_leg_mass", str(PUPPER_CONFIG.LEG_MASS))
    filedata = filedata.replace("pupper_frame_inertia", str(pupper_frame_inertia))
    filedata = filedata.replace("pupper_module_inertia", str(pupper_module_inertia))
    filedata = filedata.replace("pupper_leg_inertia", str(pupper_leg_inertia))
    filedata = filedata.replace("pupper_leg_radius", str(pupper_leg_radius))
    filedata = filedata.replace("pupper_half_size", str(pupper_half_size))
    filedata = filedata.replace("pupper_leg_fb", str(PUPPER_CONFIG.LEG_FB))
    filedata = filedata.replace("pupper_leg_lr", str(PUPPER_CONFIG.LEG_LR))
    filedata = filedata.replace("pupper_leg_geom", str(pupper_leg_geom))
    filedata = filedata.replace("pupper_start_position", str(pupper_start_position))
    filedata = filedata.replace("pupper_force_geom", str(pupper_force_geom))
    filedata = filedata.replace("pupper_hip_box", str(pupper_hip_box))
    filedata = filedata.replace("pupper_hip_offset", str(PUPPER_CONFIG.HIP_OFFSET))
    filedata = filedata.replace(
        "pupper_abduction_offset", str(PUPPER_CONFIG.ABDUCTION_OFFSET)
    )
    filedata = filedata.replace("pupper_l1_length", str(PUPPER_CONFIG.LEG_L1))
    filedata = filedata.replace("pupper_l2_length", str(PUPPER_CONFIG.LEG_L2))
    filedata = filedata.replace("pupper_l1_geom", str(pupper_leg_geom))
    filedata = filedata.replace("pupper_l2_geom", str(pupper_leg_geom))

    # Sensor noise
    filedata = filedata.replace("pupper_accel_noise", str(pupper_accel_noise))
    filedata = filedata.replace("pupper_gyro_noise", str(pupper_gyro_noise))
    filedata = filedata.replace("pupper_encoder_noise", str(pupper_encoder_noise))
    filedata = filedata.replace(
        "pupper_encoder_vel_noise", str(pupper_encoder_vel_noise)
    )
    filedata = filedata.replace("pupper_force_noise", str(pupper_force_noise))

    # Write the xml file
    with open(OUT_FILE, "w") as file:
        file.write(filedata)
