xhost + local:$USER >/dev/null 2>&1

docker run --rm \
    --privileged \
    -v /dev/bus/usb:/dev/bus/usb \
    --device-cgroup-rule='c 189:* rmw' \
    -e DISPLAY=$DISPLAY \
    -e "HOSTNAME=$(cat /etc/hostname)" \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    -v ./data:/app/data:rw \
    -v ./certs:/app/certs:ro \
    -v ./configs:/app/configs:ro \
    --network host \
    ghcr.io/fftai/teleoperation:depthai-latest \
    python -m teleoperation.main --config-name teleop_gr1 sim=true
