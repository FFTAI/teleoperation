# FROM 192.168.3.15:9595/base/python:3.11-22.04
FROM yuxianggao/python:3.11-22.04

RUN apt-get update && apt-get install -y --no-install-recommends \
    wget build-essential cmake pkg-config libjpeg-dev libtiff5-dev libavcodec-dev libavformat-dev libswscale-dev libv4l-dev libxvidcore-dev libx264-dev libgtk2.0-dev libgtk-3-dev libatlas-base-dev gfortran git \
    && apt-get clean && rm -rf /var/lib/apt/lists/*

ADD depthai_dependencies.sh .
RUN ./depthai_dependencies.sh

RUN pip install --no-cache-dir depthai-sdk
