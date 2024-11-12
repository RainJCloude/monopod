#!/bin/bash

ASSET_PATH="~/GymEnvs/prisma_walker_isaaclab/asset/"
ASSET_PATH=$(eval echo "$ASSET_PATH")

URDF_PATH="$ASSET_PATH/urdf/prisma_walker.urdf"
USD_PATH="$ASSET_PATH/usd_converted/prisma_walker_usd.usd"

XACRO_PATH="$ASSET_PATH/../../prisma_walker_ros2/src/monopod/urdf/prisma_walker.urdf.xacro"


# You can override the default values by passing arguments to the script
ASSET_PATH=${2:-$ASSET_PATH} 

ROD_LENGTH=${1:-0.5}

echo "Asset path is will be stored in: $ASSET_PATH"



xacro -o "$URDF_PATH" "$XACRO_PATH" rod_length_arg:=$ROD_LENGTH
 
 
# Step 1: Check if the URDF file exists
if [[ ! -f "$URDF_PATH" ]]; then
    echo "Error: URDF file '$URDF_PATH' not found!"
    exit 1
fi

# Step 2: Check if the output directory exists
if [[ ! -d "$(dirname "$USD_PATH")" ]]; then
    echo "Error: Output directory for USD file does not exist!"
    exit 1
fi

# Step 3: Run the Python conversion script
python3 ~/Isaac/IsaacLab/source/standalone/tools/convert_urdf.py "$URDF_PATH" "$USD_PATH" --merge-joints --make-instanceable

