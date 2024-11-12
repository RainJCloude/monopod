

import omni.isaac.lab.sim as sim_utils
from omni.isaac.lab.sim.spawners.materials.physics_materials_cfg import RigidBodyMaterialCfg

from omni.isaac.lab.actuators import ActuatorNetLSTMCfg, DCMotorCfg, ImplicitActuatorCfg, IdealPDActuatorCfg
from omni.isaac.lab.assets.articulation import ArticulationCfg
from omni.isaac.lab.assets.rigid_object import RigidObjectCfg
import math
##
# Configuration - Actuators.
##
import os

current_file_path = os.path.abspath(__file__)
current_dir = os.path.dirname(current_file_path)
path = current_dir + "/prisma_walker_4_link/prisma_walker_afshin.usd"


SIMPLE_ACTUATOR_CFG = ImplicitActuatorCfg(
    joint_names_expr=["giunto_1", "giunto_2", "giunto_3"],
    effort_limit={"giunto_1": 20.0, "giunto_2": 20.0, "giunto_3": 2.0},
    velocity_limit= math.degrees(10.18),
    stiffness={"giunto_1": 10, "giunto_2": 10, "giunto_3": 10.0},
    damping={"giunto_1": 0.8, "giunto_2": 0.8, "giunto_3": 0.1},
)


PRISMA_WALKER_CFG = ArticulationCfg(
    spawn=sim_utils.UsdFileCfg(
        ##usd_path="/home/prisma-lab-ws/Scrivania/Omniverse/IsaacLab/source/IsaacPrismaWalker/prisma_walker/asset/prisma_walker_4_link/prisma_walker_afshin.usd",
        usd_path = path,
        activate_contact_sensors=True,
        rigid_props=sim_utils.RigidBodyPropertiesCfg(
            rigid_body_enabled=True,
            max_linear_velocity=1000.0,
            max_angular_velocity=1000.0,
            max_depenetration_velocity=100.0,
            enable_gyroscopic_forces=True,
        ),
        articulation_props=sim_utils.ArticulationRootPropertiesCfg(
            enabled_self_collisions=False,
            solver_position_iteration_count=4,
            solver_velocity_iteration_count=0,
            sleep_threshold=0.005,
            stabilization_threshold=0.001,
        ),
        # collision_props=sim_utils.CollisionPropertiesCfg(contact_offset=0.02, rest_offset=0.0),
    ),
    init_state=ArticulationCfg.InitialStateCfg(
        pos=(0.0, 0.0, 0.35), #se spawnato più in basso saltella
        joint_pos={
            "giunto_1": 0.0,
            "giunto_2": 0.0,
            "giunto_3": 0.0,
        },
    ),
    actuators={"legs": SIMPLE_ACTUATOR_CFG},
    soft_joint_pos_limit_factor=0.95,
)




STEP_CFG = RigidObjectCfg(
    prim_path="/World/envs/env_.*/tennisball",
    spawn=sim_utils.CuboidCfg(
        size=[10, 10, 0.1],
        rigid_props=sim_utils.RigidBodyPropertiesCfg( #RigidBodyProp
            kinematic_enabled=False,
            disable_gravity=False,
            enable_gyroscopic_forces=True,
            solver_position_iteration_count=8,
            solver_velocity_iteration_count=8,
            sleep_threshold=0.005,
            stabilization_threshold=0.0025,
            max_depenetration_velocity=1000.0,
        ),
        mass_props=sim_utils.MassPropertiesCfg(mass=100), #RigidBodyProp
        collision_props=sim_utils.CollisionPropertiesCfg( #RigidBodyProp
            collision_enabled=True,
            contact_offset=0.001,
            rest_offset=0.0,
            torsional_patch_radius=0.01,
            min_torsional_patch_radius=0.005
        ),#Shape CFG Attributes
        visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(1.0, 0.0, 0.0)),
        physics_material=RigidBodyMaterialCfg(
            static_friction=1.0,
            dynamic_friction=0.9,
            restitution=0.12, ##stick the objects each other
            improve_patch_friction=True,
            friction_combine_mode="average",
            restitution_combine_mode="average",
            compliant_contact_stiffness=0.0,
            compliant_contact_damping=0.0
        ),
    ),
    init_state=RigidObjectCfg.InitialStateCfg(
        pos=(3.5, 0.0, 1.5),
        rot=(1.0, 0.0, 0.0, 0.0),
        lin_vel=(-4.5, 0.0, 2.5),
        ang_vel=(0.0, 0.0, 0.0),
    ),
)
