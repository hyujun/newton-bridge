# =============================================================================
# newton-bridge — standalone Newton Physics + ROS 2 Jazzy bridge
# =============================================================================
#   - Ubuntu 24.04 (Noble) because ROS 2 Jazzy officially targets Noble and
#     its Python 3.12 is already >= Newton's 3.11+ floor (guide §9: avoids
#     the imgui_bundle issue seen on 3.10).
#   - venv created with --system-site-packages so apt-installed ros-jazzy-rclpy
#     stays visible after 'source /opt/newton-venv/bin/activate'.
#   - Entrypoint sources /opt/ros/jazzy/setup.bash before delegating to CMD.
# =============================================================================

# Ubuntu 24.04 only has 12.9.x and 13.x runtime tags published; 12.9.1 is the
# closest match to the cu128 PyTorch wheels installed below (12.x ABI line).
FROM nvidia/cuda:12.9.1-runtime-ubuntu24.04

ENV DEBIAN_FRONTEND=noninteractive \
    PYTHONDONTWRITEBYTECODE=1 \
    PYTHONUNBUFFERED=1 \
    LANG=en_US.UTF-8 \
    LC_ALL=en_US.UTF-8

# -- locale + base tooling ----------------------------------------------------
RUN apt-get update && apt-get install -y --no-install-recommends \
        locales \
        ca-certificates \
        curl \
        gnupg \
        software-properties-common \
        sudo \
    && locale-gen en_US.UTF-8 \
    && update-locale LANG=en_US.UTF-8 LC_ALL=en_US.UTF-8 \
    && rm -rf /var/lib/apt/lists/*

# -- ROS 2 Jazzy apt repo -----------------------------------------------------
RUN add-apt-repository universe \
    && curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
        -o /usr/share/keyrings/ros-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu noble main" \
        | tee /etc/apt/sources.list.d/ros2.list > /dev/null

# -- system packages: ROS 2 + Newton GL/EGL deps ------------------------------
RUN apt-get update && apt-get install -y --no-install-recommends \
        # ROS 2 Jazzy (headless ros-base is enough; no GUI from ROS side)
        ros-jazzy-ros-base \
        ros-jazzy-rmw-fastrtps-cpp \
        ros-jazzy-sensor-msgs \
        ros-jazzy-std-srvs \
        ros-jazzy-std-msgs \
        ros-jazzy-rosgraph-msgs \
        ros-jazzy-geometry-msgs \
        ros-jazzy-rclpy \
        python3-colcon-common-extensions \
        python3-argcomplete \
        # Newton guide §4 — OpenGL / rendering deps (+ libgles2 for EGL headless §9)
        libgl1 \
        libegl1 \
        libgles2-mesa-dev \
        libglib2.0-0 \
        libxrandr2 \
        libxinerama1 \
        libxcursor1 \
        libxi6 \
        libxkbcommon0 \
        # python toolchain
        python3 \
        python3-dev \
        python3-pip \
        python3-venv \
        git \
    && rm -rf /var/lib/apt/lists/*

# -- python venv (--system-site-packages keeps rclpy importable) --------------
RUN python3 -m venv --system-site-packages /opt/newton-venv
ENV PATH="/opt/newton-venv/bin:${PATH}" \
    VIRTUAL_ENV="/opt/newton-venv"

RUN pip install --no-cache-dir --upgrade pip setuptools wheel

# -- Newton Physics (full extras per request) ---------------------------------
#   - examples: sim + importers + viewer (viewer needs GL deps above)
#   - torch-cu12: PyTorch CUDA 12.8 wheels for RL policy inference
#   - notebook: Jupyter + Rerun visualizer
#   - dev: test / lint tooling
RUN pip install --no-cache-dir \
        "newton[examples,notebook,dev]"

# torch-cu12 lives on PyTorch's CUDA wheel index — install in a separate layer
# so a re-pull doesn't re-download the ~2GB torch wheels unless this line changes.
RUN pip install --no-cache-dir \
        "newton[torch-cu12]" \
        --extra-index-url https://download.pytorch.org/whl/cu128

# -- newton_bridge package (editable install; src/ bind-mounted at runtime) ---
# Copy pyproject + package so `pip install -e .` resolves the package layout.
# The bind-mount in docker/compose.yml replaces /workspace/newton-bridge/src
# at runtime, so host edits are reflected without rebuilding.
WORKDIR /workspace/newton-bridge
COPY pyproject.toml ./
COPY src ./src
RUN pip install --no-cache-dir -e .

# -- entrypoint: source ROS 2 then exec CMD -----------------------------------
COPY docker/entrypoint.sh /usr/local/bin/entrypoint.sh
RUN chmod +x /usr/local/bin/entrypoint.sh

WORKDIR /workspace
ENTRYPOINT ["/usr/local/bin/entrypoint.sh"]
CMD ["bash"]
