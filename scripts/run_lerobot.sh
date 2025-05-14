#!/bin/bash

POLICY=$1
CKPT=$2

shift 2

HF_HOME=/home/fftai/Data python -m teleoperation.eval \
    --config-name eval_lerobot \
    policy=$POLICY \
    robot.instance.namespace="gr/daq" \
    eval.rerun_enabled=true \
    eval.rerun_endpoint=192.168.31.184:9876 \
    policy.instance.pretrained_path=$CKPT \
    ${@}
