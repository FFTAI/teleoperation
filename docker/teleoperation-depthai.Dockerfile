# ARG BUILDER_IMAGE=192.168.3.32/base/fourier_hardware:latest
# ARG BASE_IMAGE=192.168.3.32/farts/depthai:3.10-22.04

ARG BUILDER_IMAGE=ghcr.io/fftai/fourier_hardware:latest
ARG BASE_IMAGE=ghcr.io/fftai/depthai:3.10-22.04

FROM ${BUILDER_IMAGE} AS builder

RUN ldconfig && mkdir -p /app/deps && \
    ldd /usr/lib/python3/dist-packages/fourier_hardware_py.cpython-310-x86_64-linux-gnu.so | awk '{print $3}' | grep -v '^(' | grep 'local' | xargs -I {} cp --dereference {} /app/deps/ 2>/dev/null || true && \
    ldd /usr/lib/python3/dist-packages/fourier_hardware_dds_py.cpython-310-x86_64-linux-gnu.so | awk '{print $3}' | grep -v '^(' | grep 'local' | xargs -I {} cp --dereference {} /app/deps/ 2>/dev/null || true \
    find /app/deps -type l -exec cp --parents --dereference {} /app/deps/ \;

FROM ${BASE_IMAGE} AS runtime
COPY --from=ghcr.io/astral-sh/uv:latest /uv /uvx /bin/

ENV UV_COMPILE_BYTECODE=1
ENV UV_LINK_MODE=copy

ARG USERNAME=farts
ARG USER_UID=1000
ARG USER_GID=$USER_UID

# Create the user and group
RUN apt-get update && \
    apt-get install -y --no-install-recommends sudo && \
    apt-get clean && rm -rf /var/lib/apt/lists/* && \
    addgroup --gid $USER_GID ${USERNAME} && \
    adduser --uid $USER_UID --gid $USER_GID --disabled-password --gecos "" ${USERNAME} && \
    echo "${USERNAME} ALL=(ALL) NOPASSWD: ALL" >> /etc/sudoers

# add user to the dialout group
RUN usermod -aG dialout $USERNAME & \
    usermod -aG input $USERNAME && \
    usermod -aG plugdev $USERNAME && \
    usermod -aG sudo $USERNAME

RUN chown -R $USERNAME:$USERNAME /opt/venv

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

# Set the non-root user as the default user
USER $USERNAME


ARG UV_CACHE_DIR=/home/${USERNAME}/.cache/uv

WORKDIR /app
RUN chown -R $USERNAME:$USERNAME /app

# COPY --chown=$USERNAME:$USERNAME dexhandpy-0.0.42-cp310-cp310-linux_x86_64.whl /app/

RUN uv venv --python /opt/venv/bin/python
RUN --mount=type=cache,target=${UV_CACHE_DIR},uid=${USER_UID},gid=${USER_GID} \
    --mount=type=bind,source=uv.lock,target=uv.lock \
    --mount=type=bind,source=pyproject.toml,target=pyproject.toml \
    uv sync --frozen --no-install-project --no-dev --extra depthai --extra cpu --group fourier -vvv

COPY --chown=$USERNAME:$USERNAME src/ /app/src/
COPY --chown=$USERNAME:$USERNAME assets/ /app/assets/
COPY --chown=$USERNAME:$USERNAME configs/ /app/configs/
COPY --chown=$USERNAME:$USERNAME pyproject.toml /app/
COPY --chown=$USERNAME:$USERNAME uv.lock /app/
COPY --chown=$USERNAME:$USERNAME README.md /app/


RUN --mount=type=cache,target=${UV_CACHE_DIR},uid=${USER_UID},gid=${USER_GID} \
    uv sync --frozen --no-dev --extra depthai --extra cpu --group fourier -vvv

RUN --mount=type=cache,target=${UV_CACHE_DIR},uid=${USER_UID},gid=${USER_GID} \
    uv pip install mujoco meshcat ischedule matplotlib==3.4.3 notebook && \
    uv pip uninstall typing

# uv pip install --no-deps --force-reinstall /app/dexhandpy-0.0.42-cp310-cp310-linux_x86_64.whl && \
# Place executables in the environment at the front of the path
ENV PATH="/app/.venv/bin:$PATH"
ENV LD_LIBRARY_PATH=/usr/local/fourier_dds_msgs/lib:/usr/local/lib:/usr/lib/x86_64-linux-gnu/:/app/.venv/lib:/app/.venv/lib/python3.10/site-packages/fourier_grx_dds/libraries/

COPY --chown=farts:farts --from=builder /usr/lib/python3/dist-packages/grx_sot_py.so /app/grx_sot_py.so
COPY --chown=farts:farts --from=builder /usr/lib/python3/dist-packages/fourier_hardware_py.cpython-310-x86_64-linux-gnu.so /app/fourier_hardware_py.cpython-310-x86_64-linux-gnu.so
COPY --chown=farts:farts --from=builder /usr/lib/python3/dist-packages/fourier_hardware_dds_py.cpython-310-x86_64-linux-gnu.so /app/fourier_hardware_dds_py.cpython-310-x86_64-linux-gnu.so
COPY --chown=farts:farts --from=builder /app/deps/ /usr/local/lib/
COPY --chown=farts:farts --from=builder /usr/local/fourier_dds_msgs/ /usr/local/fourier_dds_msgs/


CMD ["source", "/app/.venv/bin/activate"]
