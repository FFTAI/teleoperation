FROM 192.168.3.15:9595/base/python:3.11-22.04

ENV DEBIAN_FRONTEND=noninteractive

RUN mv /etc/apt/sources.list /etc/apt/sources.list.bak
RUN cat <<EOF > /etc/apt/sources.list

EOF

RUN apt-get update \
    && apt-get install -y --no-install-recommends \
    libusb-1.0-0 \
    libudev-dev \
    apt-transport-https \
    ca-certificates \
    curl \
    software-properties-common \
    && rm -rf /var/lib/apt/lists/*

RUN mkdir -p /etc/apt/keyrings && \
    curl -sSf https://librealsense.intel.com/Debian/librealsense.pgp | tee /etc/apt/keyrings/librealsense.pgp > /dev/null

RUN echo "deb [signed-by=/etc/apt/keyrings/librealsense.pgp] https://librealsense.intel.com/Debian/apt-repo `lsb_release -cs` main" | \
    RUN tee /etc/apt/sources.list.d/librealsense.list

RUN apt-get update \
    && apt-get install -y --no-install-recommends \
    librealsense2-dkms \
    librealsense2-utils \
    && rm -rf /var/lib/apt/lists/*
