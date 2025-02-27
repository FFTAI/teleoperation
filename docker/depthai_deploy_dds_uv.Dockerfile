FROM 192.168.3.15:9595/farts/depthai:latest
# FROM yuxianggao/depthai:latest

COPY --from=ghcr.io/astral-sh/uv:latest /uv /uvx /bin/

ENV UV_COMPILE_BYTECODE=1
ENV UV_LINK_MODE=copy

RUN apt-get update && apt-get install -y --no-install-recommends \
    libtinyxml2-9 \
    && apt-get clean && rm -rf /var/lib/apt/lists/*

WORKDIR /app

RUN uv venv --python /opt/venv/bin/python
RUN --mount=type=cache,target=/root/.cache/uv \
    --mount=type=bind,source=uv.lock,target=uv.lock \
    --mount=type=bind,source=pyproject.toml,target=pyproject.toml \
    uv sync --frozen --no-install-project --no-dev --extra depthai --extra cpu --group fourier -vvv

COPY src/ /app/src/
COPY assets/ /app/assets/
COPY configs/ /app/configs/
COPY pyproject.toml /app/
COPY uv.lock /app/
COPY README.md /app/

RUN --mount=type=cache,target=/root/.cache/uv \
    uv sync --frozen --no-dev --extra depthai --extra cpu --group fourier -vvv

# RUN --mount=type=cache,target=/root/.cache/uv \
#     uv pip install fourier-grx-dds==0.2.7a0
RUN uv pip uninstall typing

# Place executables in the environment at the front of the path
ENV PATH="/app/.venv/bin:$PATH"

# RUN --mount=type=cache,target=/root/.cache/pip pip install --upgrade pip && \
#     pip install fourier-grx-dds==0.2.7a0 -i https://mirrors.tuna.tsinghua.edu.cn/pypi/web/simple && \
#     pip install -e ".[fourier,depthai]" --default-timeout=100 -i https://mirrors.tuna.tsinghua.edu.cn/pypi/web/simple --extra-index-url https://download.pytorch.org/whl/cpu && \
#     pip uninstall -y typing

RUN ln -s /app/.venv/lib/python3.11/site-packages/fourier_grx_dds/libraries/libfastcdr.so.2.2.5 /app/.venv/lib/python3.11/site-packages/fourier_grx_dds/libraries/libfastcdr.so.2 && \
    ln -s /app/.venv/lib/python3.11/site-packages/fourier_grx_dds/libraries/libfastdds.so.3.1.0 /app/.venv/lib/python3.11/site-packages/fourier_grx_dds/libraries/libfastdds.so.3.1

ENV LD_LIBRARY_PATH=/usr/lib/x86_64-linux-gnu/:/app/.venv/lib:/app/.venv/lib/python3.11/site-packages/fourier_grx_dds/libraries/

CMD ["source", "/app/.venv/bin/activate"]
