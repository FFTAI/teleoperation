#!/bin/bash

robot_type=$1

echo "Running Data Collection..."
echo "Machine ID: ${machine_id}"
echo "Robot Type: ${robot_type}"
# echo "Namespace: gr/daq-${machine_id}"

if [[ ! "$robot_type" =~ ^(gr1t1|gr1t2|gr2t2)$ ]]; then
    echo "Invalid robot type. Please use gr1t1, gr1t2, or gr2t2."
    exit 1
fi
echo "-----------------------------------"

docker run --rm -it --name grx \
    --net=host \
    -v ${PWD}:/app \
    192.168.3.32/grx/grx:1.0.0a20 \
    run "./${robot_type}.yaml" --namespace "gr/daq"
