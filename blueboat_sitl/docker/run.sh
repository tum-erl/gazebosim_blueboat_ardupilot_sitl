#!/usr/bin/env bash

XAUTH=/tmp/.docker.xauth
if [ ! -f $XAUTH ]
then
    xauth_list=$(xauth nlist $DISPLAY)
    xauth_list=$(sed -e 's/^..../ffff/' <<< "$xauth_list")
    if [ ! -z "$xauth_list" ]
    then
        echo "$xauth_list" | xauth -f $XAUTH nmerge -
    else
        touch $XAUTH
    fi
    chmod a+r $XAUTH
fi

local_gz_ws="/home/luke/gazebosim_blueboat_ardupilot_sitl/gz_ws"
local_SITL_Models="/home/luke/gazebosim_blueboat_ardupilot_sitl/SITL_Models"
docker run -it \
    --rm \
    --name blueboat_sitl \
    -e DISPLAY \
    -e QT_X11_NO_MITSHM=1 \
    -e XAUTHORITY=$XAUTH \
    -e NVIDIA_VISIBLE_DEVICES=all \
    -e NVIDIA_DRIVER_CAPABILITIES=all \
    -v "$XAUTH:$XAUTH" \
    -v "/tmp/.X11-unix:/tmp/.X11-unix" \
    -v "/etc/localtime:/etc/localtime:ro" \
    -v "/dev/input:/dev/input" \
    -v "$local_gz_ws:/home/blueboat_sitl/gz_ws" \
    -v "$local_SITL_Models:/home/blueboat_sitl/SITL_Models" \
    --privileged \
    --security-opt seccomp=unconfined \
    --network host \
    --gpus all \
    blueboat_sitl:latest




