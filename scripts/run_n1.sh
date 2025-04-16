#!/bin/bash

xhost + local:$USER >/dev/null 2>&1

HOST=$1
PROMPT=$2

echo "Running N1 inference..."
echo "Host: ${HOST}"
echo "Prompt: ${PROMPT}"
shift 2
echo "-----------------------------------"

docker run --rm -it --name n1 \
    --privileged \
    -v /dev/bus/usb:/dev/bus/usb \
    --device-cgroup-rule='c 189:* rmw' \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    -e DISPLAY=$DISPLAY \
    --network=host \
    --ipc=host \
    -e HYDRA_FULL_ERROR=1 \
    192.168.3.32/farts/depthai-deploy-dds:gr00t \
    python -m teleoperation.eval \
    --config-name eval_n1 \
    cpu.affinity=[0,1,2,3,4,5,6,7,8,9,10,11,12,13] \
    robot=gr1t1_legacy \
    robot.instance.namespace="gr/daq" \
    hand=fourier_dexpilot_dhx \
    camera=oak_97 \
    policy.instance.host="${HOST}" \
    eval.rerun_enabled=false \
    rerun_endpoint: 192.168.31.184:9876 \
    eval.prompt="${PROMPT}" \
    $@
