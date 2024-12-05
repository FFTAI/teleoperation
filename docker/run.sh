# docker run --rm \
#     --privileged \
#     -v /dev/bus/usb:/dev/bus/usb \
#     --device-cgroup-rule='c 189:* rmw' \
#     -e DISPLAY=$DISPLAY \
#     -v /tmp/.X11-unix:/tmp/.X11-unix \
#     -v ./depthai-python:/depthai \
#     192.168.3.15:9595/farts/depthai:latest \
#     python /depthai/examples/ColorCamera/rgb_preview.py

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
    192.168.3.15:9595/farts/depthai-deploy:latest \
    python -m teleoperation.main --config-name teleop_gr1 sim=true

# docker run -it --rm \
#     -v /dev:/dev \
#     --device-cgroup-rule "c 81:* rmw" \
#     --device-cgroup-rule "c 189:* rmw" \
#     192.168.3.15:9595/base/pyrealsense:latest rs-depth
