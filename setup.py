# Copyright (c) 2022-2024, The ORBIT Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Installation script for the 'omni.isaac.lab_tasks' python package."""

import itertools
import os
import toml

from setuptools import setup

# Obtain the extension data from the extension.toml file
EXTENSION_PATH = os.path.dirname(os.path.realpath(__file__))

# Installation operation
setup(
    name="prisma-walker-isaaclab-package",
    author="tizi a caso",
    maintainer="Si mantiene in piedi",
    maintainer_email="mittalma@ethz.ch",
    include_package_data=True,
    python_requires=">=3.7",
    packages=["prisma_walker_isaaclab"],
    classifiers=[
        "Natural Language :: English",
        "Programming Language :: Python :: 3.10",
        "Isaac Lab :: 2024.1.0-hotfix.1",
    ],
    zip_safe=False,
)

 
