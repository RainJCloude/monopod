# Copyright (c) 2022-2024, The ORBIT Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

from __future__ import annotations

import math
from dataclasses import MISSING


import omni.isaac.lab.sim as sim_utils
from omni.isaac.lab.assets import ArticulationCfg, AssetBaseCfg
from omni.isaac.lab.envs import ManagerBasedRLEnvCfg
from omni.isaac.lab.managers import CurriculumTermCfg as CurrTerm
from omni.isaac.lab.managers import EventTermCfg as EventTerm
from omni.isaac.lab.managers import ObservationGroupCfg as ObsGroup
from omni.isaac.lab.managers import ObservationTermCfg as ObsTerm
from omni.isaac.lab.managers import RewardTermCfg as RewTerm
from omni.isaac.lab.managers import SceneEntityCfg
from omni.isaac.lab.managers import TerminationTermCfg as DoneTerm
from omni.isaac.lab.scene import InteractiveSceneCfg
from omni.isaac.lab.sensors import ContactSensorCfg, RayCasterCfg, patterns
from omni.isaac.lab.terrains import TerrainImporterCfg
from omni.isaac.lab.utils import configclass
from omni.isaac.lab.utils.noise import AdditiveUniformNoiseCfg as Unoise
from omni.isaac.lab.assets.rigid_object import RigidObjectCfg

import prisma_walker_isaaclab.mdp as prisma_walker_mdp
import omni.isaac.lab_tasks.manager_based.locomotion.velocity.mdp as mdp

from .asset.prisma_walker import PRISMA_WALKER_CFG, STEP_CFG
##
# Pre-defined configs
##
from omni.isaac.lab.terrains.config.rough import ROUGH_TERRAINS_CFG  # isort: skip


##
# Scene definition
##

prisma_walker = True
@configclass
class MySceneCfg(InteractiveSceneCfg):
    """Configuration for the terrain scene with a legged robot."""

    # ground terrain
    terrain = TerrainImporterCfg(
        prim_path="/World/ground",
        terrain_type="generator",
        terrain_generator=ROUGH_TERRAINS_CFG,
        max_init_terrain_level=5,
        collision_group=-1,
        physics_material=sim_utils.RigidBodyMaterialCfg(
            friction_combine_mode="multiply",
            restitution_combine_mode="multiply",
            static_friction=1.0,
            dynamic_friction=1.0,
        ),
        visual_material=sim_utils.MdlFileCfg(
            mdl_path="{NVIDIA_NUCLEUS_DIR}/Materials/Base/Architecture/Shingles_01.mdl",
            project_uvw=True,
        ),
        debug_vis=False,
    )
    # robots
    robot: ArticulationCfg = MISSING
    step: RigidObjectCfg = MISSING
    # sensors
    height_scanner = RayCasterCfg(
        prim_path="{ENV_REGEX_NS}/Robot/base",
        offset=RayCasterCfg.OffsetCfg(pos=(0.0, 0.0, 20.0)),
        attach_yaw_only=True,
        pattern_cfg=patterns.GridPatternCfg(resolution=0.1, size=[1.6, 1.0]),
        debug_vis=False,
        mesh_prim_paths=["/World/ground"],
    ) 
    contact_forces = ContactSensorCfg(prim_path="{ENV_REGEX_NS}/Robot/.*", history_length=3, track_air_time=True)
    # lights
    light = AssetBaseCfg(
        prim_path="/World/light",
        spawn=sim_utils.DistantLightCfg(color=(0.75, 0.75, 0.75), intensity=3000.0),
    )
    sky_light = AssetBaseCfg(
        prim_path="/World/skyLight",
        spawn=sim_utils.DomeLightCfg(color=(0.13, 0.13, 0.13), intensity=1000.0),
    )


 

##
# MDP settings
##


@configclass
class CommandsCfg:
    foot_velocity = mdp.UniformVelocityCommandCfg(
        asset_name="robot",
        resampling_time_range=(10.0, 10.0),
        rel_standing_envs=0.02,
        rel_heading_envs=1.0,
        heading_command=True,
        heading_control_stiffness=0.5,
        debug_vis=True,
        ranges=mdp.UniformVelocityCommandCfg.Ranges(
            lin_vel_x=(0.0, 1.0), lin_vel_y=(0.0, 0.0), ang_vel_z=(-1.0, 1.0), heading=(-math.pi, math.pi)
        ),
    )


@configclass
class ActionsCfg:
    joint_pos = prisma_walker_mdp.JointPositionActionCfg(asset_name="robot", joint_names=[".*"], scale=0.05, use_default_offset=True)


@configclass
class ObservationsCfg:

    @configclass
    class PolicyCfg(ObsGroup):
        """Observations for policy group."""
        # observation terms (order preserved)
        base_lin_vel = ObsTerm(func=mdp.base_lin_vel, noise=Unoise(n_min=-0.1, n_max=0.1))
        base_ang_vel = ObsTerm(func=mdp.base_ang_vel, noise=Unoise(n_min=-0.2, n_max=0.2))
        joint_pos = ObsTerm(func=mdp.joint_pos, noise=Unoise(n_min=-0.01, n_max=0.01))
        joint_vel = ObsTerm(func=mdp.joint_vel, noise=Unoise(n_min=-1.5, n_max=1.5))
        actions = ObsTerm(func=mdp.last_action)
        variable_phase = ObsTerm(func=prisma_walker_mdp.phase_variable)

        def __post_init__(self):
            self.enable_corruption = True
            self.concatenate_terms = True

    # observation groups
    policy: PolicyCfg = PolicyCfg()


@configclass
class EventCfg:
   # startup
    physics_material = EventTerm(
        func=mdp.randomize_rigid_body_material,
        mode="startup",
        params={
            "asset_cfg": SceneEntityCfg("robot", body_names=".*"),
            "static_friction_range": (0.8, 0.8),
            "dynamic_friction_range": (0.6, 0.6),
            "restitution_range": (0.0, 0.0),
            "num_buckets": 64,
        },
    )

    """ add_base_mass = EventTerm(
        func=mdp.randomize_rigid_body_mass,
        mode="startup",
        params={"asset_cfg": SceneEntityCfg("robot", body_names="base"), "mass_distribution_params": (-10, 10.0)},
    ) """

    # reset
    """base_external_force_torque = EventTerm(
        func=mdp.apply_external_force_torque,
        mode="reset",
        params={
            "asset_cfg": SceneEntityCfg("robot", body_names="base_link"),
            "force_range": (0.0, 0.0),
            "torque_range": (-0.0, 0.0),
        },
    )"""

    #Reset the root of the robot
    reset_base = EventTerm(
        func=mdp.reset_root_state_uniform,
        mode="reset",
        params={
            "pose_range": {"x": (-0.01, 0.01), "y": (-0.01, 0.01), "yaw": (0, 0)},
            "velocity_range": {"x": (0, 0), "y": (0, 0), "yaw": (0, 0),
                "roll": (0,0),
                "pitch": (0,0),
                "yaw": (0,0)
                }
         },
    )
    
    #Reset the joint configuration
    reset_robot_joints = EventTerm(
        func=prisma_walker_mdp.reset_joints_by_file,
        mode="reset",
        params={
            "position_range_dynamixel": (-0.7, 0.7),
        },
    ) 

    # interval
    """push_robot = EventTerm(
        func=mdp.push_by_setting_velocity,
        mode="interval",
        interval_range_s=(10.0, 15.0),
        params={"velocity_range": {"x": (-0.5, 0.5), "y": (-0.5, 0.5)}},
    )"""


@configclass
class RewardsCfg:
    """Reward terms for the MDP."""
    
     # -- penalties
    dof_torques_l2 = RewTerm(func=mdp.joint_torques_l2, 
        weight=-5.0e-4,
    )
    lin_vel = RewTerm(func=mdp.track_lin_vel_xy_exp, 
        weight=1,
        params={"command_name": "base_velocity", "std": math.sqrt(0.25)}
    )
    pitchPenalty = RewTerm(
        func=prisma_walker_mdp.pitchPenalty,
        weight=-0.01,
        params={"sensor_cfg": SceneEntityCfg("contact_forces", body_names="base_link")},
    )
    imitation_joint = RewTerm(
        func=prisma_walker_mdp.imitation_joint,
        weight=1.5,
        params={"asset_cfg": SceneEntityCfg("robot", body_names="piede_interno")},
    )
    foot_vel_error = RewTerm(
        func=prisma_walker_mdp.track_foot_vel,
        weight=1.5,
        params={"command_name": "foot_velocity", "std": math.sqrt(0.25), "asset_cfg": SceneEntityCfg("robot", body_names="piede_interno"),},
    )
    """sliding = RewTerm(
        func=prisma_walker_mdp.slippage_base,
        weight=-0.0,
        params={"sensor_cfg": SceneEntityCfg("contact_forces", body_names=["base_link", "piede_interno"])}
    )
    sliding_central_foot = RewTerm(
        func=prisma_walker_mdp.slippage_central_foot,
        weight=-0.1,
        params={
        "body_cfg": SceneEntityCfg("robot", body_names = "piede_interno"),
        }
    )
    clearance = RewTerm(
        func=prisma_walker_mdp.clearance,
        weight=0.0,
        params={
        "body_cfg": SceneEntityCfg("robot", body_names = "piede_interno")
        }
    )"""



@configclass
class TerminationsCfg:
    """Termination terms for the MDP."""
    time_out = DoneTerm(func=mdp.time_out, time_out=True)
    pitch_condition = DoneTerm(
        func=prisma_walker_mdp.pitchTermination,
        params={"sensor_cfg": SceneEntityCfg("contact_forces", body_names="base_link"), "offset": 25.0},
    )
    #penalty_for_big_error = DoneTerm(func=prisma_walker_mdp.big_erros)
    

@configclass
class CurriculumCfg:
    """Curriculum terms for the MDP."""

    terrain_levels = None #CurrTerm(func=mdp.terrain_levels_vel) Currently not found this callable obj


##
# Environment configuration
##


@configclass
class LocomotionVelocityRoughEnvCfg(ManagerBasedRLEnvCfg):
    """Configuration for the locomotion velocity-tracking environment."""
    followReferenceTraj = False
    # Scene settings
    scene: MySceneCfg = MySceneCfg(num_envs=2048, env_spacing=2.5)
    # Basic settings
    observations: ObservationsCfg = ObservationsCfg()
    actions: ActionsCfg = ActionsCfg()
    commands: CommandsCfg = CommandsCfg()
    # MDP settings
    rewards: RewardsCfg = RewardsCfg()
    terminations: TerminationsCfg = TerminationsCfg()
    events: EventCfg = EventCfg()
    curriculum: CurriculumCfg = CurriculumCfg()

    scene.height_scanner.prim_path = "{ENV_REGEX_NS}/Robot/base_link"
    scene.robot = PRISMA_WALKER_CFG.replace(prim_path="{ENV_REGEX_NS}/Robot")
    scene.step = STEP_CFG

    def __post_init__(self):
        """Post initialization."""
        # general settings
        self.viewer.lookat = [0.0, 0.0, 0.0] #look at the center of the world
        self.viewer.eye = [2, 2, 2]
        self.viewer.resolution = [500, 500]
        self.decimation = 2
        self.episode_length_s = 8  
        # simulation settings
        self.sim.dt = 0.005
        self.sim.disable_contact_processing = True
        self.sim.physics_material = self.scene.terrain.physics_material
        # update sensor update periods
        # we tick all the sensors based on the smallest update period (physics update period)
        if self.scene.height_scanner is not None:
            self.scene.height_scanner.update_period = self.decimation * self.sim.dt
        if self.scene.contact_forces is not None:
            self.scene.contact_forces.update_period = self.sim.dt

        # check if terrain levels curriculum is enabled - if so, enable curriculum for terrain generator
        # this generates terrains with increasing difficulty and is useful for training
        if getattr(self.curriculum, "terrain_levels", None) is not None:
            if self.scene.terrain.terrain_generator is not None:
                self.scene.terrain.terrain_generator.curriculum = True
        else:
            if self.scene.terrain.terrain_generator is not None:
                self.scene.terrain.terrain_generator.curriculum = False

@configclass
class PrismaWalkerFlatEnvCfg(LocomotionVelocityRoughEnvCfg):
    def __post_init__(self):
        # post init of parent
        super().__post_init__()
        # switch robot to anymal-d
        
        if(self.followReferenceTraj):
            self.terminations.penalty_for_big_error = None

        self.scene.terrain.terrain_type = "plane"
        self.scene.terrain.terrain_generator = None
        self.scene.env_spacing = 2.5
        # no height scan
        #self.scene.height_scanner = None
        self.observations.policy.height_scan = None
        # no terrain curriculum
        self.curriculum.terrain_levels = None
        # remove conditions on contact sensors removal
 


@configclass
class PrismaWalkerEnvCfg_PLAY(PrismaWalkerFlatEnvCfg):
    def __post_init__(self):
        # post init of parent
        super().__post_init__()

        # make a smaller scene for play
        self.scene.num_envs = 1
        self.scene.env_spacing = 2.5
        # spawn the robot randomly in the grid (instead of their terrain levels)
        self.scene.terrain.max_init_terrain_level = None
        # reduce the number of terrains to save memory
        if self.scene.terrain.terrain_generator is not None:
            self.scene.terrain.terrain_generator.num_rows = 5
            self.scene.terrain.terrain_generator.num_cols = 5
            self.scene.terrain.terrain_generator.curriculum = False

        # disable randomization for play
        self.observations.policy.enable_corruption = False
        # remove random pushing
        self.randomization.base_external_force_torque = None
        self.randomization.push_robot = None
