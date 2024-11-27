# FROM 192.168.3.15:9595/base/python:3.11-22.04
FROM yuxianggao/python:3.11-22.04

COPY src/ /app/src/
COPY assets/ /app/assets/
COPY pyproject.toml /app/
COPY pdm.lock /app/
COPY README.md /app/

WORKDIR /app

RUN pip install --upgrade --no-cache-dir  pip && \
    pip install --no-cache-dir -e ".[fourier,depthai]" --extra-index-url https://download.pytorch.org/whl/cpu && \
    pip uninstall -y typing
