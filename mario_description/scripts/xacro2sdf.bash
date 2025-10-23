#!/usr/bin/env bash
# This script converts xacro (URDF variant) into SDF for `mario_description` package

SCRIPT_DIR="$(cd "$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")" &>/dev/null && pwd)"
XACRO_PATH="$(dirname "${SCRIPT_DIR}")/urdf/mario.urdf.xacro"
SDF_PATH="$(dirname "${SCRIPT_DIR}")/mario/model.sdf"
TMP_URDF_PATH="/tmp/mario_tmp.urdf"

# Arguments for xacro
XACRO_ARGS=(
    name:=mario
    collision_arm:=true
    ros2_control:=true
    ros2_control_plugin:=gz
    ros2_control_command_interface:=position
    gazebo_preserve_fixed_joint:=false
)

# Remove old SDF file
rm "${SDF_PATH}" 2>/dev/null

# Process xacro into URDF, then convert URDF to SDF and edit the SDF to use relative paths for meshes
xacro "${XACRO_PATH}" "${XACRO_ARGS[@]}" -o "${TMP_URDF_PATH}" &&
ign sdf -p "${TMP_URDF_PATH}" | sed "s/model:\/\/mario_description\///g" >"${SDF_PATH}" &&
echo "Created new ${SDF_PATH}"

# Remove temporary URDF file
rm "${TMP_URDF_PATH}" 2>/dev/null
