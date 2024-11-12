# Copyright (c) 2022-2024, The lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Common functions that can be used to enable reward functions.

The functions can be passed to the :class:`omni.isaac.lab.managers.RewardTermCfg` object to include
the reward introduced by the function.
"""

from __future__ import annotations
import numpy
import torch
from typing import TYPE_CHECKING

from omni.isaac.lab.assets import Articulation, RigidObject
from omni.isaac.lab.managers import SceneEntityCfg
from omni.isaac.lab.sensors import ContactSensor
from omni.isaac.lab.utils.math import matrix_from_quat
import omni.isaac.lab.utils.math as math_utils
from collections.abc import Sequence
import math
from prisma_walker_isaaclab import rlTaskImit
if TYPE_CHECKING:
    from omni.isaac.lab.envs import ManagerBasedRLEnv
    from omni.isaac.lab.envs import ManagerBasedEnv

"""
General.
"""


class bcolors:
    HEADER = '\033[95m'
    OKBLUE = '\033[94m'
    OKCYAN = '\033[96m'
    OKGREEN = '\033[92m'
    WARNING = '\033[93m'
    FAIL = '\033[91m'
    ENDC = '\033[0m'
    BOLD = '\033[1m'
    UNDERLINE = '\033[4m'

 
def phase_variable(env: ManagerBasedEnv, asset_cfg: SceneEntityCfg = SceneEntityCfg("robot")) -> torch.Tensor:
    return env.phase_variable_observed.unsqueeze(dim=1)


def imitation_joint(env: rlTaskImit, asset_cfg: SceneEntityCfg = SceneEntityCfg("robot", body_names = "piede_interno")) -> torch.Tensor:
    
    asset: Articulation = env.scene[asset_cfg.name]

    """increment indices"""
    increment = torch.randint(1, 2, size = (len(env.index_imit),), device = 'cuda:0')
    
    """raised error on physix if i incremented the index_imit tensor over the size"""
    idx = (env.index_imit + increment) >= env.idx_len #1818
    env.index_imit[idx] = 1
    env.index_imit[~idx] += increment[:len(env.index_imit[~idx])]
    
    #Phase variable update
    row_indices = torch.arange(env.index_imit.size(0))
    env.phase_variable_observed = env.reference_phase_variable[row_indices, torch.clamp(env.index_imit, min=0, max=1816).tolist()]  
    a = env.phase_variable_observed - env.phase_variable_
    #References
    val_m1 = env.m1JointPos[env.index_imit.tolist()]
    val_m2 = env.m2JointPos[env.index_imit.tolist()]
     
    error_m1 = val_m1 - asset.data.joint_pos[:,0]  #joint_pos is (num_envs, num_joints)
    error_m2 = val_m2 - asset.data.joint_pos[:,1]
    return torch.exp(-2*(error_m1*error_m1 + error_m2*error_m2)) 



def track_foot_vel(
        env: rlTaskImit,
        command_name: str, std: float,
        asset_cfg: SceneEntityCfg = SceneEntityCfg("robot", body_names = "piede_interno"), 
        ) -> torch.Tensor:
    
    asset: Articulation = env.scene[asset_cfg.name]

    foot_vel = asset.data.body_state_w[:, asset_cfg.body_ids, 7:10]
    foot_vel = foot_vel.squeeze(dim=1)
    foot_vel_xz = torch.stack((foot_vel[:,0], foot_vel[:,2]), dim=1)

    reference_mean_vel = torch.rand(0, 1, device="cuda:0")

    foot_vel_error = torch.square(torch.abs(env.command_manager.get_command(command_name)[:, :2]) - torch.abs(foot_vel_xz))
    foot_vel_error[:, 1] = 0 #not interested in error on y-direction

    foot_vel_error_sum = torch.sum(
        foot_vel_error,
        dim=1,
    )

    """foot_vel_error = torch.sum(
        torch.square(reference_mean_vel - torch.mean(foot_vel_xz)),
        dim=1,
    )"""
     
    
    return torch.exp(-foot_vel_error_sum / std**2)




def slippage_base(env: ManagerBasedRLEnv, sensor_cfg: SceneEntityCfg) -> torch.Tensor:

    # extract the used quantities (to enable type-hinting)
    asset: RigidObject = env.scene["robot"]
    contact_sensor: ContactSensor = env.scene.sensors[sensor_cfg.name]
    
    is_base_in_contact = torch.norm(contact_sensor.data.net_forces_w[:, sensor_cfg.body_ids[0], :], dim = -1) > 0.0

    is_foot_in_air = torch.norm(contact_sensor.data.net_forces_w[:, sensor_cfg.body_ids[1], :], dim = -1) == 0.0

    base_contact_foot_in_air = is_base_in_contact * is_foot_in_air 

    #(num_envs) true if at least one the lateral feet touch the ground
    base_lin_vel_xy = torch.norm(asset.data.body_lin_vel_w[:, sensor_cfg.body_ids[0], :2], dim=-1)
    #(num_envs)

    penalty = torch.square(base_lin_vel_xy*base_contact_foot_in_air)

    return penalty


def slippage_central_foot(env: ManagerBasedRLEnv,
             body_cfg: SceneEntityCfg = SceneEntityCfg("robot", body_names = "piede_interno")):

    asset: Articulation | RigidObject = env.scene[body_cfg.name]
    contact_sensor: ContactSensor = env.scene.sensors["contact_forces"]

    center_foot_tan_vel = torch.norm(asset.data.body_lin_vel_w[:, body_cfg.body_ids, :2], dim = -1)
    
    contact_force = torch.norm(contact_sensor.data.net_forces_w[:, body_cfg.body_ids,:], dim=-1)
   
    in_contact = contact_force > 0.0

    tangential_vel_contact_foot = center_foot_tan_vel.squeeze(dim=1) * in_contact.squeeze(dim=1)

    return torch.square(tangential_vel_contact_foot)
 

def clearance(env: ManagerBasedRLEnv,
             body_cfg: SceneEntityCfg = SceneEntityCfg("robot", body_names = "piede_interno")):

    asset: Articulation | RigidObject = env.scene[body_cfg.name]
    #contact_sensor: ContactSensor = env.scene.sensors["contact_forces"]
     
    foot_vel = asset.data.body_state_w[:, body_cfg.body_ids, 7:10]
    print(foot_vel.shape)
    foot_vel_x = foot_vel[:,0]
    #(num_envs)
 
    env.foot_height_buffer += foot_height 
    first_contact = contact_sensor.compute_first_contact(env.step_dt)[:, body_cfg.body_ids]
    reward = env.foot_height_buffer * first_contact.squeeze(dim=1)
    
    """At each contact, reset the buffer"""
    idx = first_contact.nonzero(as_tuple=False)
    env.foot_height_buffer[idx] = 0.0
       
    return reward



def lin_vel_z_l2(env: ManagerBasedRLEnv, asset_cfg: SceneEntityCfg = SceneEntityCfg("robot")) -> torch.Tensor:
    """Penalize z-axis base linear velocity using L2-kernel."""
    # extract the used quantities (to enable type-hinting)
    asset: RigidObject = env.scene[asset_cfg.name] #self.scene["robot"]
    foot_vel = asset.data.body_state_w[:, 4, 7:10]
    return torch.square(asset.data.root_lin_vel_b[:, 2])





def reset_joints_by_file(
    env: ManagerBasedEnv,
    env_ids: torch.Tensor,
    position_range_dynamixel: tuple[float, float],
    asset_cfg: SceneEntityCfg = SceneEntityCfg("robot"),
):
    
    #print(f"{bcolors.FAIL}C'è stato il reset{bcolors.FAIL}")
    asset: Articulation = env.scene[asset_cfg.name]

    joint_pos = asset.data.default_joint_pos[env_ids].clone()
    joint_vel = asset.data.default_joint_vel[env_ids].clone()
    
    normal_tensor = torch.normal(0, 1, size=(1,))
    
    """if(normal_tensor > 0.8):
        new_motor_idx = torch.randint(0, env.idx_len, size = (len(env_ids),), device = 'cuda:0')
    else:
        new_motor_idx = torch.randint(0, 400, size = (len(env_ids),), device = 'cuda:0')"""
    new_motor_idx = torch.randint(0, env.idx_len, size = (len(env_ids),), device = 'cuda:0')

    env.index_imit[env_ids] = new_motor_idx
    
    val_m1_for_env = env.m1JointPos[env.index_imit[env_ids].tolist()]
    val_m2_for_env = env.m2JointPos[env.index_imit[env_ids].tolist()]

    # bias these values randomly
    joint_pos_dynamixel = math_utils.sample_uniform(*position_range_dynamixel, len(env_ids), joint_pos.device)
    joint_pos = torch.stack((
        val_m1_for_env,
        val_m2_for_env,
        joint_pos_dynamixel), dim=1)
    
    # clamp joint pos to limits
    joint_pos_limits = asset.data.soft_joint_pos_limits[env_ids]
    joint_pos = joint_pos.clamp_(joint_pos_limits[..., 0], joint_pos_limits[..., 1])
    env.foot_height_buffer[env_ids] = 0

    # set into the physics simulation
    asset.write_joint_state_to_sim(joint_pos, joint_vel, env_ids=env_ids)


"""norm between the 3 axis, the max between the histories, look if the firs element is greater than the threshold"""
"""the output of torch.max is a column vector. Torch.any() of a column vector would return a boolean, (false, if all the elements are false, true if ust once is true)"""
"""But doing Torch.any(tensor, dim=1) means to check only along the rows if among all the elements there 
   is at least a True. If I do: torch.any(tensor) where tensor = [true,
                                                                 false,
                                                                 true]
    it would return only one True (because in the tensor there is at least one True). But if I perform:
    torch.any(tensor, dim = 1), it returns tensor = [true,
                                                    false,
                                                    true]
    because it is looking if for each row (iterating among the columns, because dim = 1) there is at least one true"""

"""Since the tensor is a col vector, doing dim=1 means that it return exectly that element for each row, 
   because it means to check if there is at least one true iterating along the columns, but in each row
   there is only one element and therefore it return the element itself"""

def pitchCondition(env: ManagerBasedRLEnv, sensor_cfg: SceneEntityCfg):

    z_axis = torch.tensor([0, 0, 1]).to('cuda:0')
    z_axis_ex = z_axis.expand(env.num_envs, -1) # is (num_envs, 3)

    bodyOrientation: RigidObject = env.scene["robot"]
    quat = bodyOrientation.data.root_state_w[:,3:7]
   
    #orientation of the root (N, 4)
    #quat = contact_sensor.data.quat_w
    """it has dimension (N,B,4)"""

    pitch = matrix_from_quat(quat)
    """it has dimension (N,B,3,3) if taken from env.scene.sensors, (N,3,3) otherwise"""
    pitch = pitch[:,:,2]
    "Take the z-orientation of the base_link"

    pitch_angle = torch.acos(torch.sum( pitch * z_axis_ex, dim = 1)) #the product with * is a dot product
    #make the product and then sum along the rows (dim=1 means columsn, sum among the element of each col)
    #column vector of angles in radiants

    pitch_angle = pitch_angle*180/math.pi
    #yaw angle is shape [num_envs, ] and contains the value of the angles:
    return pitch_angle

def pitchTermination(env: ManagerBasedRLEnv, sensor_cfg: SceneEntityCfg, offset):

    pitch_angle = pitchCondition(env, sensor_cfg)
    pitch_unsqueezed = torch.unsqueeze(pitch_angle, dim = 1) #turn from [num_envs] into [num_envs, 1]
    #Now I can perform torch.any along the cols, and obtain a vector of true or false if the single element of each row violate or not that condition
 
    #print(f"{bcolors.WARNING}pitch angle: {bcolors.ENDC}", torch.any(yaw_unsqueezed > 50, dim = 1).cpu().numpy(), "Value of the pitch angle: ", yaw_angle.cpu().numpy())
    return torch.any(pitch_unsqueezed > offset, dim = 1)

def pitchPenalty(env: ManagerBasedRLEnv, sensor_cfg: SceneEntityCfg):

    pitch_angle = pitchCondition(env, sensor_cfg)

    return torch.square(pitch_angle)


def big_erros(env: ManagerBasedRLEnv):

    idx_to_punish = env.error_exceeded.nonzero(as_tuple=False)

    env.error_exceeded[idx_to_punish] = False

    penalty_for_big_error = torch.zeros(env.num_envs, device='cuda:0')

    penalty_for_big_error[idx_to_punish] = 1
    
    #print(f"{bcolors.HEADER}termination for error: {bcolors.OKGREEN}", penalty_for_big_error.bool().cpu().numpy())
    return penalty_for_big_error.bool()


