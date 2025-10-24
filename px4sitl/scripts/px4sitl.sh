#!/usr/bin/env bash

set -e
set -o pipefail


PX4SITL_WORLD_SDF_NAME=default
PX4SITL_DRONE_MODELS=()

INSTALL_ROOT=$(realpath "$(dirname "$0")/../")
PX4SITL_SIM_RESOURCES=${INSTALL_ROOT}/px4/Tools/simulation/gazebo-classic/sitl_gazebo-classic/

SERVICE=px4sitl_gz_gui

while [[ $# -gt 0 ]]
do
    case $1 in
        -H|--headless)      SERVICE="px4sitl_ros";                      shift;;
        -w|--world)         PX4SITL_WORLD_SDF_NAME=$2;                  shift; shift;;
        -l|--list_worlds)   ls -1 "${PX4SITL_SIM_RESOURCES}/worlds";    exit 0;;
        -d|--drone)
            PX4SITL_DRONE_MODELS+=("$2");
            PX4SITL_DRONE_POSES+=("$3");
            shift; shift; shift;;
    esac
done


if [ ${#PX4SITL_DRONE_MODELS[@]} -eq 0 ]
then
    PX4SITL_DRONE_MODELS+=(4001)
    PX4SITL_DRONE_POSES+=("0,0,0,0,0,0");
fi


cdinit.sh start "${SERVICE}" \
    PATH="${PATH}:${INSTALL_ROOT}/px4/bin" \
    GZ_SIM_RESOURCE_PATH="${GZ_SIM_RESOURCE_PATH}:${PX4SITL_SIM_RESOURCES}/models:${PX4SITL_SIM_RESOURCES}/worlds" \
    GZ_SIM_SYSTEM_PLUGIN_PATH="${GZ_SIM_SYSTEM_PLUGIN_PATH}:${INSTALL_ROOT}/lib/px4_gz_plugins/" \
    GZ_SIM_SERVER_CONFIG_PATH="${INSTALL_ROOT}/px4/Tools/simulation/gz/server.config" \
    PX4SITL_WORLD_SDF_PATH="${PX4SITL_SIM_RESOURCES}/worlds/${PX4SITL_WORLD_SDF_NAME}.sdf"


INSTANCE=0
for DRONE in "${PX4SITL_DRONE_MODELS[@]}";
do
#PX4_HOME_LAT=51.1788
#PX4_HOME_LON=-1.8263
#PX4_HOME_ALT=101
#PX4_SIM_SPEED_FACTOR=2

    cdinit.sh start "px4sitl_px4@${INSTANCE}" \
        PX4_GZ_WORLD="${PX4SITL_WORLD_SDF_NAME}" \
        PX4_GZ_MODEL_POSE="${PX4SITL_DRONE_POSES[${INSTANCE}]}" \
        PX4_SYS_AUTOSTART="${DRONE}"

    ((INSTANCE=INSTANCE+1))
done
