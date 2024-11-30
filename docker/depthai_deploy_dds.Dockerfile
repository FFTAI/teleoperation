FROM 192.168.3.15:9595/farts/depthai:latest
# FROM yuxianggao/depthai:latest 

RUN apt-get update && apt-get install -y --no-install-recommends \
    libtinyxml2-9 \
    && apt-get clean && rm -rf /var/lib/apt/lists/*

COPY src/ /app/src/
COPY assets/ /app/assets/
# COPY configs/ /app/configs/
COPY pyproject.toml /app/
COPY pdm.lock /app/
COPY README.md /app/

WORKDIR /app

RUN --mount=type=cache,target=/root/.cache/pip pip install --upgrade pip && \
    pip install fourier-grx-dds==0.2.3a0 -i https://mirrors.tuna.tsinghua.edu.cn/pypi/web/simple && \
    pip install -e ".[fourier,depthai]" --default-timeout=100 -i https://mirrors.tuna.tsinghua.edu.cn/pypi/web/simple --extra-index-url https://download.pytorch.org/whl/cpu && \
    pip uninstall -y typing

ENV LD_LIBRARY_PATH=/opt/venv/lib/python3.11/site-packages/fourier_grx_dds/libraries/:/usr/lib/x86_64-linux-gnu/

