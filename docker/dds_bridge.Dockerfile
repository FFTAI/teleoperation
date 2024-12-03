FROM 192.168.3.15:9595/base/python:3.11-22.04

RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential cmake pkg-config libtinyxml2-9 \
    && apt-get clean && rm -rf /var/lib/apt/lists/*

RUN --mount=type=cache,target=/root/.cache/pip pip install --upgrade pip && \
    pip install fourier-grx-dds==0.2.4a0 -i https://mirrors.tuna.tsinghua.edu.cn/pypi/web/simple

RUN rm /opt/venv/lib/python3.11/site-packages/fourier_grx_dds/libraries/libfastcdr.so.2 && \
    rm /opt/venv/lib/python3.11/site-packages/fourier_grx_dds/libraries/libfastdds.so.3.1 && \
    ln -s /opt/venv/lib/python3.11/site-packages/fourier_grx_dds/libraries/libfastcdr.so.2.2.5 /opt/venv/lib/python3.11/site-packages/fourier_grx_dds/libraries/libfastcdr.so.2 && \
    ln -s /opt/venv/lib/python3.11/site-packages/fourier_grx_dds/libraries/libfastdds.so.3.1.0 /opt/venv/lib/python3.11/site-packages/fourier_grx_dds/libraries/libfastdds.so.3.1

ENV LD_LIBRARY_PATH=/usr/lib/x86_64-linux-gnu/:/opt/venv/lib/python3.11/site-packages/fourier_grx_dds/libraries/

WORKDIR /app