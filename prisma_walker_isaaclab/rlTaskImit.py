import gym
import torch

from omni.isaac.lab.envs import ManagerBasedRLEnv
from omni.isaac.lab.envs import ManagerBasedRLEnvCfg
from omni.isaac.lab.utils.math import sample_uniform

import numpy as np
import os
import sys

class rlTaskIm(ManagerBasedRLEnv):
      def __init__(self, cfg: ManagerBasedRLEnvCfg, render_mode: str | None = None, **kwargs):
            """super take 2 parameter, the name of the classe and the name of the method which you are extending in the son class
            It can be implemented as super(rlTaskIm, self) to say that you are extending just this method"""
            #os.getcwd() take the path of the folder in which you launch the command
            
            use_reduced_dataset = False
            
            if(use_reduced_dataset):
                  m1 = np.loadtxt(os.getcwd() + "/prisma_walker_isaaclab/pos_m1_15s.txt", dtype=float)
                  m2 = np.loadtxt(os.getcwd() + "/prisma_walker_isaaclab/pos_m2_15s.txt", dtype=float)         
            else:
                  m1 = np.loadtxt(os.getcwd() + "/prisma_walker_isaaclab/pos_m1_18s.txt", dtype=float)
                  m2 = np.loadtxt(os.getcwd() + "/prisma_walker_isaaclab/pos_m2_18s.txt", dtype=float)
            
            self.idx_len = len(m1) 

            pos_x = np.loadtxt(os.getcwd() + "/prisma_walker_isaaclab/asset/posa_x.txt", dtype=float)
            pos_z = np.loadtxt(os.getcwd() + "/prisma_walker_isaaclab/asset/posa_y.txt", dtype=float)
            
            single_distances = 0
            self.traj = []
            for i in range(self.idx_len -1):
                  if(False):
                        single_distances += np.abs(pos_x[i+1] - pos_x[i])
                  else:
                        single_distances += np.sqrt(np.square(pos_x[i+1] - pos_x[i]) + np.square(pos_z[i+1] - pos_z[i]) )
                  self.traj.append(single_distances)

            self.tot_len = self.traj[-1]
            phi = self.traj/self.tot_len
            torch_phase_variable = torch.from_numpy(phi).float().to('cuda:0')
            
            self.reference_phase_variable = torch.zeros((cfg.scene.num_envs, torch_phase_variable.shape[0]), device='cuda:0')
            self.reference_phase_variable[:] = torch_phase_variable
            self.phase_variable_observed = torch.zeros(cfg.scene.num_envs, device='cuda:0')
            
            np.set_printoptions(threshold=sys.maxsize)

            self.m1JointPos = torch.from_numpy(m1).float().to('cuda:0')
            self.m2JointPos = torch.from_numpy(m2).float().to('cuda:0')

            self.pos_x = torch.from_numpy(pos_x).float().to('cuda:0')
            self.pos_z = torch.from_numpy(pos_z).float().to('cuda:0')

            self.foot_pos_x = torch.zeros(cfg.scene.num_envs, device='cuda:0') 
            self.old_pos_x = torch.zeros(cfg.scene.num_envs, device='cuda:0') 
            self.foot_pos_z = torch.zeros(cfg.scene.num_envs, device='cuda:0') 
            self.old_pos_z = torch.zeros(cfg.scene.num_envs, device='cuda:0') 
            self.phase_variable_ = torch.zeros(cfg.scene.num_envs, device='cuda:0') 
            self.old_phase_variable_ = torch.zeros(cfg.scene.num_envs, device='cuda:0') 
            self.single_distances = torch.zeros(cfg.scene.num_envs, device='cuda:0')

            #self.device still doesn't exist
            """The super must go afterwards because otherwise the baseEnv calls the actionManagar
            which call the joint_actions.py when this attribute still doesn't exist"""

            self.index_imit = torch.zeros(cfg.scene.num_envs, device='cuda:0')

            #obs manager is called by the base_env whereas the reward manager is initialized by the rl_task_envs
            self.history_len = 5
            self.joint_pos_history = torch.zeros(cfg.scene.num_envs, self.history_len, 3, device='cuda:0')
            self.joint_vel_history = torch.zeros_like(self.joint_pos_history)      

            self.foot_height_buffer = torch.zeros(cfg.scene.num_envs, device='cuda:0')
            self.error_exceeded = torch.zeros(cfg.scene.num_envs, device='cuda:0')      

            super().__init__(cfg, render_mode, **kwargs)
            #reset_all = np.linspace(0, cfg.scene.num_envs-5, cfg.scene.num_envs, dtype=int).tolist()
             
            """for event in self.event_manager._mode_term_cfgs["reset"]:
                  
                 event.func(self, reset_all)
            """
            
            self.reset_all_joints(cfg.scene.num_envs)
           
            #self.event_manager._mode_term_cfgs["reset"][1].func(self, reset_all, (-1.57, 1.57))
            #print(self.event_manager._mode_term_cfgs["reset"][0].func)
            self._joint_pos_target_sim = torch.zeros(cfg.scene.num_envs, 3, device='cuda:0')
            self.idxa = torch.zeros(cfg.scene.num_envs, device='cuda:0') 
 


      def step(self, action):
 
            #use this with one environment
            #print(torch.norm(self.scene.sensors["contact_forces"].data.net_forces_w, dim=-1).nonzero())
            #print(self.scene["robot"].data.soft_joint_pos_limits)
            
            if(self.cfg.followReferenceTraj):

                  # print(self.idxa)
                  self._joint_pos_target_sim[:, 0] = self.m1JointPos[self.idxa.tolist()]
                  self._joint_pos_target_sim[:, 1] = self.m2JointPos[self.idxa.tolist()]

                  self.idxa += 1
                  indici_ecceduti = self.idxa >= 1818
                  self.idxa[indici_ecceduti.nonzero(as_tuple=False)] = 1
                  #if(self.idx == 1400):
                  #elf._reset_idx(self.idxa.nonzero(as_tuple=False))
            
            # process actions
            self.action_manager.process_action(action)
            # perform physics stepping
            for _ in range(self.cfg.decimation):
                  # set actions into buffers
                  self.action_manager.apply_action()
                  # set actions into simulator     
                  if(self.cfg.followReferenceTraj):
                        self.scene["robot"].root_physx_view.set_dof_position_targets(self._joint_pos_target_sim, self.scene["robot"]._ALL_INDICES)
                  else:
                        self.scene.write_data_to_sim()
                  #simulate
                  self.sim.step(render=False)
                  # update buffers at sim dt
                  self.scene.update(dt=self.physics_dt)
            # perform rendering if gui is enabled
            if self.sim.has_gui() or self.sim.has_rtx_sensors():
                  self.sim.render()
            
            # post-step:
            # -- update env counters (used for curriculum generation)
            self.episode_length_buf += 1  # step in current episode (per env)
            self.common_step_counter += 1  # total step (common for all envs)
            # -- check terminations
            self.reset_buf = self.termination_manager.compute()
            self.reset_terminated = self.termination_manager.terminated
            self.reset_time_outs = self.termination_manager.time_outs
            # -- reward computation

            self.foot_pos_x = self.scene["robot"].data.body_state_w[:, 4, 0]
            self.foot_pos_z = self.scene["robot"].data.body_state_w[:, 4, 2]
            self.single_distances += torch.sqrt(torch.square(self.foot_pos_x - self.old_pos_x) + torch.square(self.foot_pos_z - self.old_pos_z))
            mask = self.single_distances >= self.tot_len
            self.single_distances[mask] = self.single_distances[mask] - self.tot_len
            self.phase_variable_ = self.single_distances/self.tot_len

            self.reward_buf = self.reward_manager.compute(dt=self.step_dt)

            # -- reset envs that terminated/timed-out and log the episode information
            reset_env_ids = self.reset_buf.nonzero(as_tuple=False).squeeze(-1)
            if len(reset_env_ids) > 0:
                  self._reset_idx(reset_env_ids)
            # -- update command
            self.command_manager.compute(dt=self.step_dt)
            # -- step interval events
            if "interval" in self.event_manager.available_modes:
                  self.event_manager.apply(mode="interval", dt=self.step_dt)
            # -- compute observations
            # note: done after reset to get the correct observations for reset envs   
            
            self.obs_buf = self.observation_manager.compute()
            
            self.old_pos_x = self.foot_pos_x.clone()
            self.old_pos_z = self.foot_pos_z.clone()
            self.old_phase_variable_ = self.phase_variable_.clone()

            # return observations, rewards, resets and extras
            return self.obs_buf, self.reward_buf, self.reset_terminated, self.reset_time_outs, self.extras

      
      """when in the step function of RLTask will be called the self._reset_idx it will call this function
      because "self" is the object of the class rlTaskImit"""
      


      def reset_all_joints(self, num_envs):

            asset: Articulation = self.scene["robot"]

            joint_pos = asset.data.default_joint_pos.clone()
            joint_vel = asset.data.default_joint_vel.clone()
            
            self.index_imit = torch.randint(0, 1817, (num_envs,), device = 'cuda:0')

            val_m1_for_env = self.m1JointPos[self.index_imit.tolist()]
            val_m2_for_env = self.m2JointPos[self.index_imit.tolist()]

            # bias these values randomly
            joint_pos_dynamixel = sample_uniform(-1.57, 1.57, num_envs, joint_pos.device)
          
            joint_pos = torch.stack((
                  val_m1_for_env,
                  val_m2_for_env,
                  joint_pos_dynamixel), dim=1)
            
            # clamp joint pos to limits
            """joint_pos_limits = asset.data.soft_joint_pos_limits[env_ids]
            joint_pos = joint_pos.clamp_(joint_pos_limits[..., 0], joint_pos_limits[..., 1])
            """

            joint_vel = torch.zeros(joint_vel.shape, device=joint_vel.device)
            # set into the physics simulation
            asset.write_joint_state_to_sim(joint_pos, joint_vel)
