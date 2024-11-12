# Copyright (c) 2022-2024, The ORBIT Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""
Script to print all the available environments in ORBIT.

The script iterates over all registered environments and stores the details in a table.
It prints the name of the environment, the entry point and the config file.

All the environments are registered in the `omni.isaac.lab_tasks` extension. They start
with `Isaac` in their name.
"""

from __future__ import annotations

"""Launch Isaac Sim Simulator first."""


from omni.isaac.lab.app import AppLauncher

# launch omniverse app
app_launcher = AppLauncher(headless=True)
simulation_app = app_launcher.app


"""Rest everything follows."""

import gymnasium as gym
from prettytable import PrettyTable

import prisma_walker
 

def main():
    """Print all environments registered in `omni.isaac.lab_tasks` extension."""
    # print all the available environments
    table = PrettyTable(["S. No.", "Task Name", "Entry Point", "Config"])
    table.title = "Available Environments in ORBIT"
    # set alignment of table columns
    table.align["Task Name"] = "l"
    table.align["Entry Point"] = "l"
    table.align["Config"] = "l"

    # count of environments
    index = 0
    # acquire all Prisma-made environments names
    for task_spec in gym.registry.values():
        if "Prisma" in task_spec.id or "Lbr_iiwa" in task_spec.id or "Prisma_Walker_A" in task_spec.id:
            # add details to table
            table.add_row([index + 1, task_spec.id, task_spec.entry_point, task_spec.kwargs["env_cfg_entry_point"]])
            # increment count
            index += 1

    print(table)


if __name__ == "__main__":
    try:
        # run the main function
        main()
    except Exception as e:
        raise e
    finally:
        # close the app
        simulation_app.close()