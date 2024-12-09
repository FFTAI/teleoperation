FROM 192.168.3.15:9595/base/python:3.11-20.04
# FROM yuxianggao/python:3.11-20.04

ENV DEBIAN_FRONTEND=noninteractive

RUN cat <<EOF > /etc/apt/sources.list
    deb https://mirrors.tuna.tsinghua.edu.cn/ubuntu/ focal main restricted universe multiverse
    # deb-src https://mirrors.tuna.tsinghua.edu.cn/ubuntu/ focal main restricted universe multiverse
    deb https://mirrors.tuna.tsinghua.edu.cn/ubuntu/ focal-updates main restricted universe multiverse
    # deb-src https://mirrors.tuna.tsinghua.edu.cn/ubuntu/ focal-updates main restricted universe multiverse
    deb https://mirrors.tuna.tsinghua.edu.cn/ubuntu/ focal-backports main restricted universe multiverse
    # deb-src https://mirrors.tuna.tsinghua.edu.cn/ubuntu/ focal-backports main restricted universe multiverse

    # 以下安全更新软件源包含了官方源与镜像站配置，如有需要可自行修改注释切换
    deb http://security.ubuntu.com/ubuntu/ focal-security main restricted universe multiverse
    # deb-src http://security.ubuntu.com/ubuntu/ focal-security main restricted universe multiverse
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

RUN mkdir -p /etc/apt/keyrings && curl -sSf https://librealsense.intel.com/Debian/librealsense.pgp | tee /etc/apt/keyrings/librealsense.pgp > /dev/null

RUN echo "deb [signed-by=/etc/apt/keyrings/librealsense.pgp] https://librealsense.intel.com/Debian/apt-repo `lsb_release -cs` main" | \
    tee /etc/apt/sources.list.d/librealsense.list

RUN apt-get update \
    && apt-get install -y --no-install-recommends \
    librealsense2-dkms \
    librealsense2-utils \
    && rm -rf /var/lib/apt/lists/*
