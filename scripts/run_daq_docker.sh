xhost + local:$USER >/dev/null 2>&1
machine_id=$(echo "$(hostname)" | sed 's/[^0-9]//g')
robot_type=$1
camera=$2
task=$3


echo "Running Data Collection..."
echo "Machine ID: ${machine_id}"
echo "Robot Type: ${robot_type}"
echo "Camera: ${camera}"
echo "Task: ${task}"
echo "Namespace: gr/daq-${machine_id}"
echo "-----------------------------------"
shift 3

echo "Additional arguments: $@"

docker run --rm -it --name daq \
    --privileged \
    -v /dev/bus/usb:/dev/bus/usb \
    --device-cgroup-rule='c 189:* rmw' \
    -e DISPLAY=$DISPLAY \
    -e "HOSTNAME=$(cat /etc/hostname)" \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    -v /mnt/Data:/app/data:rw \
    -v ~/.certs:/app/certs:ro \
    -v ~/.farts/outputs:/app/outputs:rw \
    --network=host \
    --ipc=host \
    -e HYDRA_FULL_ERROR=1 \
    ghcr.io/fftai/teleoperation:depthai-latest \
    python -m teleoperation \
    --config-name daq \
    robot=${robot_type}_legacy \
    robot.instance.namespace="gr/daq" \
    task_name="${task}"_$(date +%m_%d) \
    camera="${camera}" \
    hand=fourier_dexpilot_dhx \
    $@
