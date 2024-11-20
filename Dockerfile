FROM ubuntu:20.04

# Replacement source
RUN apt update -y
RUN mv /etc/apt/sources.list /etc/apt/sources.list.bak
RUN cat <<EOF > /etc/apt/sources.list
    deb http://mirrors.ustc.edu.cn/ubuntu/ focal main restricted universe multiverse
    deb http://mirrors.ustc.edu.cn/ubuntu/ focal-updates main restricted universe multiverse
    deb http://mirrors.ustc.edu.cn/ubuntu/ focal-backports main restricted universe multiverse
    deb http://mirrors.ustc.edu.cn/ubuntu/ focal-security main restricted universe multiverse
EOF


RUN apt update -y
RUN mkdir -p /data/teleoperation_dds
COPY teleoperation_dds /data/teleoperation_dds

# dds dynamic library
RUN ln -s /data/teleoperation_dds/libraries/libfastcdr.so.2.2.5 /data/teleoperation_dds/libraries/libfastcdr.so.2
RUN ln -s /data/teleoperation_dds/libraries/libfastdds.so.3.1.0 /data/teleoperation_dds/libraries/libfastdds.so.3.1
RUN echo "\nexport LD_LIBRARY_PATH=/data/teleoperation_dds/libraries:\$LD_LIBRARY_PATH" >> ~/.bashrc
ENV LD_LIBRARY_PATH=/data/teleoperation_dds/libraries:$LD_LIBRARY_PATH

# Install python3.11.10 and its dependencies
RUN apt update -y && apt-get install software-properties-common -y
RUN add-apt-repository ppa:deadsnakes/ppa && apt update -y && apt install python3.11 python3.11-distutils curl python3.11-tk -y
RUN rm /usr/bin/python3 && ln -s /usr/bin/python3.11 /usr/bin/python3
RUN curl -sS https://bootstrap.pypa.io/get-pip.py | python3.11
RUN apt install libtinyxml2-dev libpython3.11-dev libx11-dev tk-dev xclip xsel -y
RUN apt update -y && apt install -y  python3-xlib libxkbfile-dev xauth
RUN python3 -m pip install pynput


# teleoperation
RUN echo "\nexport DISPLAY=:1" >> ~/.bashrc
RUN echo "\nln -sf /usr/bin/python3 /usr/bin/python" >> ~/.bashrc
RUN apt install gcc -y
RUN cd /data/teleoperation_dds/ && pip install -e '.[fourier,realsense]' -i https://pypi.tuna.tsinghua.edu.cn/simple
RUN pip uninstall typing -y

# cv2
RUN apt install libgl1 -y

# Realscense
RUN mkdir -p /etc/apt/keyrings
RUN curl -sSf https://librealsense.intel.com/Debian/librealsense.pgp | tee /etc/apt/keyrings/librealsense.pgp > /dev/null
RUN apt-get install apt-transport-https
RUN echo "deb [signed-by=/etc/apt/keyrings/librealsense.pgp] https://librealsense.intel.com/Debian/apt-repo `lsb_release -cs` main" | \
    tee /etc/apt/sources.list.d/librealsense.list
RUN apt-get update
RUN apt-get install librealsense2-dkms -y
RUN apt-get install librealsense2-utils -y

WORKDIR /data/teleoperation_dds
