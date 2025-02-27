docker run  --rm -it --privileged -v ./src:/app/src -v /tmp/.X11-unix:/tmp/.X11-unix  -v /dev/bus/usb:/dev/bus/usb --device-cgroup-rule='c 189:* rmw' 192.168.3.15:9595/farts/depthai-deploy-dds:uv python \
    $@
