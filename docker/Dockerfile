FROM ubuntu:22.04

ENV DEBIAN_FRONTEND=noninteractive

WORKDIR /app

# PX4 SITL build tools and runtime libraries for the PyQt/GStreamer application.
RUN apt-get update && apt-get install -y --no-install-recommends \
    astyle \
    bc \
    build-essential \
    ca-certificates \
    ccache \
    cmake \
    cppcheck \
    curl \
    file \
    g++ \
    gcc \
    gdb \
    git \
    gnupg \
    lcov \
    libasound2 \
    libdbus-1-3 \
    libfontconfig1 \
    libegl1 \
    libgl1 \
    libgles2 \
    libglib2.0-0 \
    libglvnd0 \
    libgstreamer-plugins-base1.0-dev \
    libglx0 \
    libnss3 \
    libssl-dev \
    libx11-xcb1 \
    libxcb-cursor0 \
    libxcb-glx0 \
    libxcb-icccm4 \
    libxcb-image0 \
    libxcb-keysyms1 \
    libxcb-randr0 \
    libxcb-render-util0 \
    libxcb-shape0 \
    libxcb-xinerama0 \
    libxcb-xkb1 \
    libxcomposite1 \
    libxdamage1 \
    libxkbcommon-x11-0 \
    libxrandr2 \
    libxml2-dev \
    libxml2-utils \
    libxtst6 \
    lsb-release \
    make \
    mesa-utils \
    ninja-build \
    pkg-config \
    python3 \
    python3-dev \
    python3-pip \
    python3-venv \
    libqt5svg5 \
    rsync \
    shellcheck \
    unzip \
    wget \
    xfce4-terminal \
    xterm \
    zip \
    gstreamer1.0-libav \
    gstreamer1.0-plugins-bad \
    gstreamer1.0-plugins-base \
    gstreamer1.0-plugins-good \
    gstreamer1.0-plugins-ugly \
    && rm -rf /var/lib/apt/lists/*

# PX4 1.16+ uses Gazebo Harmonic (gz-sim8), distributed by OSRF for Jammy.
RUN wget -q https://packages.osrfoundation.org/gazebo.gpg \
        -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable jammy main" \
        > /etc/apt/sources.list.d/gazebo-stable.list \
    && apt-get update \
    && apt-get install -y --no-install-recommends gz-harmonic libunwind-dev \
    && rm -rf /var/lib/apt/lists/*

# PX4 Gazebo plugins use the OpenCV C++ SDK at configure and link time.
RUN apt-get update \
    && apt-get install -y --no-install-recommends libopencv-dev \
    && rm -rf /var/lib/apt/lists/*

ARG PYTORCH_VARIANT=cpu

LABEL org.sw-uav.torch-variant="${PYTORCH_VARIANT}"

ENV VIRTUAL_ENV=/opt/venv \
    PATH=/opt/venv/bin:${PATH} \
    SW_UAV_TORCH_VARIANT=${PYTORCH_VARIANT} \
    QT_XCB_GL_INTEGRATION=xcb_glx \
    QTWEBENGINE_DISABLE_SANDBOX=1 \
    PIP_DISABLE_PIP_VERSION_CHECK=1 \
    PIP_NO_CACHE_DIR=1

RUN python3 -m venv "${VIRTUAL_ENV}" \
    && python -m pip install --upgrade pip wheel

# Copy requirement manifests first so source edits do not invalidate this layer.
COPY docker/requirements.txt /tmp/app-requirements.txt
COPY docker/requirements-px4.txt /tmp/px4-requirements.txt

RUN case "${PYTORCH_VARIANT}" in \
        cpu|cu118|cu124|cu126) ;; \
        *) echo "Unsupported PYTORCH_VARIANT: ${PYTORCH_VARIANT}" >&2; exit 1 ;; \
    esac \
    && python -m pip install -r /tmp/px4-requirements.txt \
    && python -m pip install \
        --index-url "https://download.pytorch.org/whl/${PYTORCH_VARIANT}" \
        --extra-index-url https://pypi.org/simple \
        torch==2.6.0 \
        torchvision==0.21.0 \
        torchaudio==2.6.0 \
    && python -m pip install -r /tmp/app-requirements.txt

# Keep frequently tweaked interactive-shell tooling after the heavy dependency
# layers so prompt/theme changes do not invalidate the PX4/Gazebo/Python cache.
RUN apt-get update \
    && apt-get install -y --no-install-recommends xdg-utils zsh \
    && rm -rf /var/lib/apt/lists/*

RUN printf '%s\n' \
    'export SW_UAV_IN_DOCKER=1' \
    'export PROMPT_DIRTRIM=3' \
    'export PS1="\[\033[1;36m\][SW-UAV docker]\[\033[0m\] \[\033[1;33m\]\w\[\033[0m\] \$ "' \
    > /etc/bash.bashrc.docker-prompt \
    && printf '%s\n' \
    '' \
    'if [ -f /etc/bash.bashrc.docker-prompt ]; then' \
    '    . /etc/bash.bashrc.docker-prompt' \
    'fi' \
    >> /etc/bash.bashrc \
    && printf '%s\n' \
    'export SW_UAV_IN_DOCKER=1' \
    'PROMPT="%F{cyan}[SW-UAV docker]%f %F{yellow}%3~%f %# "' \
    'alias ll="ls -alF"' \
    'alias px4dir="cd /app/dependencies/PX4-Autopilot"' \
    > /root/.zshrc

RUN git config --global --add safe.directory '*'

COPY . .

CMD ["sleep", "infinity"]
