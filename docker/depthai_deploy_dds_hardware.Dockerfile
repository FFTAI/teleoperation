FROM 192.168.3.15:9595/base/fourier_hardware:v2.4 AS builder
# RUN apt update && apt install -y build-essential ldd findutils

RUN ldconfig && mkdir -p /app/deps && \
    ldd /usr/local/lib/fourier_hardware_py.cpython-310-x86_64-linux-gnu.so | awk '{print $3}' | grep -v '^(' | grep 'local' | xargs -I {} cp --dereference {} /app/deps/ 2>/dev/null || true && \
    ldd /usr/local/lib/fourier_hardware_dds_py.cpython-310-x86_64-linux-gnu.so | awk '{print $3}' | grep -v '^(' | grep 'local' | xargs -I {} cp --dereference {} /app/deps/ 2>/dev/null || true \
    find /app/deps -type l -exec cp --parents --dereference {} /app/deps/ \;

FROM 192.168.3.15:9595/farts/depthai:3.10-22.04 AS runtime
COPY --from=ghcr.io/astral-sh/uv:latest /uv /uvx /bin/

ENV UV_COMPILE_BYTECODE=1
ENV UV_LINK_MODE=copy

RUN apt-get update && apt-get install -y --no-install-recommends \
    libtinyxml2-9  git cmake build-essential \
    libboost-system-dev libboost-timer-dev libboost-program-options-dev \
    libboost-thread-dev libboost-test-dev pkg-config libeigen3-dev \
    libboost-filesystem-dev libassimp-dev nlohmann-json3-dev \
    liboctomap-dev liburdfdom-headers-dev liburdfdom-dev \
    libboost-python1.74-dev \
    mesa-utils libgl1-mesa-dev libglib2.0-dev openssl libssl-dev \
    libasio-dev libtinyxml2-dev clang pybind11-dev \
    libprotoc-dev libzmq3-dev \
    liblz4-dev libzstd-dev \
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
RUN --mount=type=cache,target=/root/.cache/uv \
    uv pip install mujoco meshcat ischedule matplotlib==3.4.3 notebook && \
    uv pip install dexhandpy==0.0.36 --index https://test.pypi.org/simple && \
    uv pip uninstall typing

# Place executables in the environment at the front of the path
ENV PATH="/app/.venv/bin:$PATH"
ENV LD_LIBRARY_PATH=/usr/local/lib:/usr/lib/x86_64-linux-gnu/:/app/.venv/lib:/app/.venv/lib/python3.10/site-packages/fourier_grx_dds/libraries/

COPY --from=builder /usr/lib/python3/dist-packages/grx_sot_py.so /app/grx_sot_py.so
COPY --from=builder /usr/lib/python3/dist-packages/fourier_hardware_py.cpython-310-x86_64-linux-gnu.so /app/fourier_hardware_py.cpython-310-x86_64-linux-gnu.so
COPY --from=builder /usr/lib/python3/dist-packages/fourier_hardware_dds_py.cpython-310-x86_64-linux-gnu.so /app/fourier_hardware_dds_py.cpython-310-x86_64-linux-gnu.so
COPY --from=builder /app/deps/ /usr/local/lib/

CMD ["source", "/app/.venv/bin/activate"]
