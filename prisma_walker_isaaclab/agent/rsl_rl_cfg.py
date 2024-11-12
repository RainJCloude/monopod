# Copyright (c) 2022-2024, The ORBIT Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

from omni.isaac.lab.utils import configclass

from omni.isaac.lab_tasks.utils.wrappers.rsl_rl import (
    RslRlOnPolicyRunnerCfg,
    RslRlPpoActorCriticCfg,
    RslRlPpoAlgorithmCfg,
)


@configclass
class PrismaWalkerRoughPPORunnerCfg(RslRlOnPolicyRunnerCfg):
    num_steps_per_env = 40
    max_iterations = 1500
    save_interval = 50
    experiment_name = "prisma_rough"
    empirical_normalization = False
    policy = RslRlPpoActorCriticCfg(
        init_noise_std=10.0,
        actor_hidden_dims=[512, 256, 128],
        critic_hidden_dims=[512, 256, 128],
        activation="elu",
    )
    algorithm = RslRlPpoAlgorithmCfg(
        value_loss_coef=1.0,
        use_clipped_value_loss=True,
        clip_param=0.2,
        entropy_coef=0.005,
        num_learning_epochs=3,
        num_mini_batches=4,
        learning_rate=1.0e-4,
        schedule="adaptive",
        gamma=0.999,
        lam=0.96,
        desired_kl=0.01,
        max_grad_norm=1.0,
    )


@configclass
class PrismaWalkerFlatPPORunnerCfg(PrismaWalkerRoughPPORunnerCfg):
    def __post_init__(self):
        super().__post_init__()

        self.max_iterations = 25000
        self.experiment_name = "prisma_walker_flat"
        self.policy.actor_hidden_dims = [128, 128]
        self.policy.critic_hidden_dims = [128, 128]
