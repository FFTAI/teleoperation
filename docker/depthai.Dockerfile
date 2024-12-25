FROM 192.168.3.15:9595/base/python:3.11-22.04
# FROM yuxianggao/python:3.11-22.04

RUN apt-get update && apt-get install -y --no-install-recommends \
    wget build-essential cmake pkg-config libjpeg-dev libtiff5-dev libavcodec-dev libavformat-dev libswscale-dev libv4l-dev libxvidcore-dev libx264-dev libgtk2.0-dev libgtk-3-dev libatlas-base-dev gfortran git jq \
    && apt-get clean && rm -rf /var/lib/apt/lists/*

ADD depthai_dependencies.sh .
RUN ./depthai_dependencies.sh

RUN pip install --no-cache-dir depthai-sdk

# disable sentry
RUN mkdir -p ~/.depthai_sdk && \
    touch ~/.depthai_sdk/config.json && \
    echo '{}' | jq '. + {"sentry": false, "sentry_dsn": "https://67bc97fb3ee947bf90d83c892eaf19fe@sentry.luxonis.com/3"}' | tee ~/.depthai_sdk/config.json >/dev/null 2>&1
