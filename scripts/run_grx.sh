#!/bin/bash

robot_type=$1

echo "Running Data Collection..."
echo "Machine ID: ${machine_id}"
echo "Robot Type: ${robot_type}"
# echo "Namespace: gr/daq-${machine_id}"
echo "-----------------------------------"

docker run --rm -it --name grx \
    --net=host \
    -v ./server_config:/app \
    192.168.3.32/grx/grx:1.0.0a20 \
    run "./${robot_type}.yaml" --namespace "gr/daq"
