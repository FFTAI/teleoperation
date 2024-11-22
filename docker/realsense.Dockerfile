ARG BASE_IMAGE=yuxianggao/python:3.11-22.04
#################################
#   Librealsense Builder Stage  #
#################################
FROM $BASE_IMAGE AS librealsense-builder

ARG LIBRS_VERSION
# Make sure that we have a version number of librealsense as argument
RUN test -n "$LIBRS_VERSION"

# To avoid waiting for input during package installation
ENV DEBIAN_FRONTEND=noninteractive

# RUN mv /etc/apt/sources.list /etc/apt/sources.list.bak
# RUN cat <<EOF > /etc/apt/sources.list
#     deb http://mirrors.ustc.edu.cn/ubuntu/ focal main restricted universe multiverse
#     deb http://mirrors.ustc.edu.cn/ubuntu/ focal-updates main restricted universe multiverse
#     deb http://mirrors.ustc.edu.cn/ubuntu/ focal-backports main restricted universe multiverse
#     deb http://mirrors.ustc.edu.cn/ubuntu/ focal-security main restricted universe multiverse
# EOF


# RUN mv /etc/apt/sources.list /etc/apt/sources.list.bak
RUN cat <<EOF > /etc/apt/sources.list
    deb https://mirrors.tuna.tsinghua.edu.cn/debian/ bookworm main contrib non-free non-free-firmware
    deb https://mirrors.tuna.tsinghua.edu.cn/debian/ bookworm-updates main contrib non-free non-free-firmware
    deb https://mirrors.tuna.tsinghua.edu.cn/debian/ bookworm-backports main contrib non-free non-free-firmware
    deb https://security.debian.org/debian-security bookworm-security main contrib non-free non-free-firmware
EOF

# Builder dependencies installation
RUN apt-get update \
    && apt-get install -qq -y --no-install-recommends \
    build-essential \
    cmake \
    git \
    libssl-dev \
    libusb-1.0-0-dev \
    pkg-config \
    libgtk-3-dev \
    libglfw3-dev \
    libgl1-mesa-dev \
    libglu1-mesa-dev \
    libudev-dev \
    curl \
    # python3 \
    # python3-dev \
    ca-certificates \
    && rm -rf /var/lib/apt/lists/*

# Download sources
WORKDIR /usr/src
RUN curl https://codeload.github.com/IntelRealSense/librealsense/tar.gz/refs/tags/v$LIBRS_VERSION -o librealsense.tar.gz
RUN tar -zxf librealsense.tar.gz \
    && rm librealsense.tar.gz
RUN ln -s /usr/src/librealsense-$LIBRS_VERSION /usr/src/librealsense

# Build and install
RUN cd /usr/src/librealsense \
    && mkdir build && cd build \
    && cmake \
    -DCMAKE_C_FLAGS_RELEASE="${CMAKE_C_FLAGS_RELEASE} -s" \
    -DCMAKE_CXX_FLAGS_RELEASE="${CMAKE_CXX_FLAGS_RELEASE} -s" \
    -DCMAKE_INSTALL_PREFIX=/opt/librealsense \
    -DBUILD_GRAPHICAL_EXAMPLES=OFF \
    -DBUILD_PYTHON_BINDINGS:bool=true \
    -DCMAKE_BUILD_TYPE=Release ../ \
    && make -j$(($(nproc)-1)) all \
    && make install

######################################
#   librealsense Base Image Stage    #
######################################
FROM ${BASE_IMAGE} AS librealsense

# Copy binaries from builder stage
COPY --from=librealsense-builder /opt/librealsense /usr/local/
COPY --from=librealsense-builder /usr/lib/python3/dist-packages/pyrealsense2 /usr/lib/python3/dist-packages/pyrealsense2
COPY --from=librealsense-builder /usr/src/librealsense/config/99-realsense-libusb.rules /etc/udev/rules.d/
COPY --from=librealsense-builder /usr/src/librealsense/config/99-realsense-d4xx-mipi-dfu.rules /etc/udev/rules.d/
ENV PYTHONPATH=$PYTHONPATH:/usr/local/lib

# Install dep packages
RUN apt-get update \
    && apt-get install -y --no-install-recommends \
    libusb-1.0-0 \
    udev \
    apt-transport-https \
    ca-certificates \
    curl \
    software-properties-common \
    && rm -rf /var/lib/apt/lists/*

# Shows a list of connected Realsense devices
CMD [ "rs-enumerate-devices", "--compact" ]
