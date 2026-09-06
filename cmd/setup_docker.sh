#!/usr/bin/env bash
set -Eeuo pipefail

# ============================================================
# Setup Docker workstation cho Ubuntu >= 20.04
#
# Bao gồm:
#   - Docker Engine
#   - Docker CLI
#   - containerd
#   - Docker Buildx
#   - Docker Compose Plugin
#   - NVIDIA Container Toolkit (tùy chọn)
#   - X11 utilities cho RViz/Gazebo
#   - Thêm user vào group docker
#   - Docker hello-world test (tùy chọn)
#   - NVIDIA GPU test (tùy chọn)
#
# Cách chạy:
#   sudo ./setup_docker.sh
#
# Ví dụ:
#   sudo INSTALL_NVIDIA=0 ./setup_docker.sh
#   sudo GPU_TEST=1 ./setup_docker.sh
# ============================================================

readonly MIN_UBUNTU_VERSION="20.04"

readonly DOCKER_GPG_URL="https://download.docker.com/linux/ubuntu/gpg"

readonly NVIDIA_GPG_URL="https://nvidia.github.io/libnvidia-container/gpgkey"
readonly NVIDIA_LIST_URL="https://nvidia.github.io/libnvidia-container/stable/deb/nvidia-container-toolkit.list"

# ------------------------------------------------------------
# Config
# ------------------------------------------------------------

# INSTALL_NVIDIA=auto:
#   Chỉ cài NVIDIA Container Toolkit khi host có NVIDIA GPU/driver hoạt động.
#   Có thể đặt 0 hoặc 1 để ghi đè việc tự phát hiện.
INSTALL_NVIDIA="${INSTALL_NVIDIA:-auto}"

# DOCKER_TEST=1:
#   Chạy hello-world để kiểm tra Docker.
DOCKER_TEST="${DOCKER_TEST:-1}"

# GPU_TEST=1:
#   Chạy container kiểm tra NVIDIA GPU.
#   Mặc định tắt vì cần pull image từ Internet.
GPU_TEST="${GPU_TEST:-0}"


# ============================================================
# Helper functions
# ============================================================

log() {
    echo
    echo "==> $*"
}

warn() {
    echo
    echo "WARNING: $*" >&2
}

package_installed() {
    dpkg -s "$1" &>/dev/null
}

user_in_docker_group() {
    id -nG "${TARGET_USER}" 2>/dev/null | grep -qw docker
}


# ============================================================
# Pre-check
# ============================================================

if [[ "${EUID}" -ne 0 ]]; then
    echo "Vui lòng chạy script bằng quyền root:"
    echo
    echo "  sudo ./setup_docker.sh"
    exit 1
fi


# ------------------------------------------------------------
# Xác định user thật sự gọi sudo
# ------------------------------------------------------------

if [[ -n "${SUDO_USER:-}" && "${SUDO_USER}" != "root" ]]; then
    TARGET_USER="${SUDO_USER}"
else
    TARGET_USER="${USER:-root}"
fi

if ! id "${TARGET_USER}" &>/dev/null; then
    echo "Không tìm thấy user '${TARGET_USER}'."
    exit 1
fi


# ------------------------------------------------------------
# Kiểm tra OS
# ------------------------------------------------------------

if [[ ! -r /etc/os-release ]]; then
    echo "Không thể xác định hệ điều hành (/etc/os-release không tồn tại)."
    exit 1
fi

# shellcheck disable=SC1091
source /etc/os-release

if [[ "${ID:-}" != "ubuntu" ]]; then
    echo "Script này chỉ hỗ trợ Ubuntu."
    echo "Hệ điều hành hiện tại: ${PRETTY_NAME:-không xác định}"
    exit 1
fi

if [[ -z "${VERSION_ID:-}" ]]; then
    echo "Không xác định được phiên bản Ubuntu."
    exit 1
fi

if dpkg --compare-versions "${VERSION_ID}" lt "${MIN_UBUNTU_VERSION}"; then
    echo "Ubuntu ${VERSION_ID} không được hỗ trợ."
    echo "Yêu cầu tối thiểu: Ubuntu ${MIN_UBUNTU_VERSION}."
    exit 1
fi


# Ubuntu 20.04 hiện đã hết hỗ trợ chính thức từ Docker CE mới.
if dpkg --compare-versions "${VERSION_ID}" lt "22.04"; then
    warn "Ubuntu ${VERSION_ID} >= ${MIN_UBUNTU_VERSION}, nhưng Docker CE hiện không còn hỗ trợ chính thức Ubuntu 20.04."
    warn "Script vẫn tiếp tục, nhưng repository Docker có thể không cung cấp Docker CE mới nhất."
fi


export DEBIAN_FRONTEND=noninteractive

case "${INSTALL_NVIDIA}" in
    auto)
        if command -v nvidia-smi &>/dev/null && nvidia-smi -L &>/dev/null; then
            INSTALL_NVIDIA=1
        else
            INSTALL_NVIDIA=0
        fi
        ;;
    0|1) ;;
    *)
        echo "INSTALL_NVIDIA phải là auto, 0 hoặc 1."
        exit 1
        ;;
esac


echo "=========================================="
echo " Docker Workstation Setup"
echo "=========================================="
echo "OS              : ${PRETTY_NAME}"
echo "Ubuntu version  : ${VERSION_ID}"
echo "Codename        : ${VERSION_CODENAME:-unknown}"
echo "Architecture    : $(dpkg --print-architecture)"
echo "Target user     : ${TARGET_USER}"
echo "INSTALL_NVIDIA  : ${INSTALL_NVIDIA}"
echo "DOCKER_TEST     : ${DOCKER_TEST}"
echo "GPU_TEST        : ${GPU_TEST}"
echo "=========================================="


# ============================================================
# Docker
# ============================================================

remove_conflicting_docker_packages() {

    log "Kiểm tra và gỡ các package Docker xung đột..."

    local conflicting_packages=(
        docker.io
        docker-doc
        docker-compose
        docker-compose-v2
        docker-buildx
        podman-docker
        containerd
        runc
    )

    local package

    for package in "${conflicting_packages[@]}"; do
        if package_installed "${package}"; then
            echo "Gỡ package: ${package}"
            apt-get remove -y "${package}"
        fi
    done
}


install_docker_engine() {

    # Nếu Docker CE + Compose + Buildx đã đầy đủ thì không cài lại.
    if package_installed docker-ce \
        && command -v docker &>/dev/null \
        && docker compose version &>/dev/null \
        && docker buildx version &>/dev/null; then

        log "Docker Engine, Compose và Buildx đã có — bỏ qua cài đặt Docker."
        return 0
    fi


    log "Cập nhật APT và cài dependency..."

    apt-get update -qq

    apt-get install -y --no-install-recommends \
        ca-certificates \
        curl \
        gnupg


    remove_conflicting_docker_packages


    # --------------------------------------------------------
    # Docker GPG key
    # --------------------------------------------------------

    log "Thêm Docker GPG key..."

    install -m 0755 -d /etc/apt/keyrings

    curl -fsSL "${DOCKER_GPG_URL}" \
        -o /etc/apt/keyrings/docker.asc

    chmod a+r /etc/apt/keyrings/docker.asc


    # --------------------------------------------------------
    # Docker repository
    # --------------------------------------------------------

    log "Thêm Docker APT repository..."

    # Xóa format .list cũ nếu script phiên bản cũ từng tạo.
    rm -f /etc/apt/sources.list.d/docker.list

    cat > /etc/apt/sources.list.d/docker.sources <<EOF
Types: deb
URIs: https://download.docker.com/linux/ubuntu
Suites: ${UBUNTU_CODENAME:-${VERSION_CODENAME}}
Components: stable
Architectures: $(dpkg --print-architecture)
Signed-By: /etc/apt/keyrings/docker.asc
EOF


    # --------------------------------------------------------
    # Install Docker
    # --------------------------------------------------------

    log "Cài Docker Engine + Compose + Buildx..."

    apt-get update -qq

    apt-get install -y --no-install-recommends \
        docker-ce \
        docker-ce-cli \
        containerd.io \
        docker-buildx-plugin \
        docker-compose-plugin
}


# ============================================================
# X11
# ============================================================

install_x11_utils() {

    log "Cài x11-xserver-utils cho RViz/Gazebo..."

    apt-get install -y --no-install-recommends \
        x11-xserver-utils
}


# ============================================================
# NVIDIA Container Toolkit
# ============================================================

install_nvidia_toolkit() {

    if [[ "${INSTALL_NVIDIA}" != "1" ]]; then
        log "INSTALL_NVIDIA=${INSTALL_NVIDIA} — bỏ qua NVIDIA Container Toolkit."
        return 0
    fi


    if ! command -v nvidia-smi &>/dev/null; then
        warn "Không tìm thấy nvidia-smi."
        warn "NVIDIA driver có thể chưa được cài trên host."
        warn "Container Toolkit vẫn được cài, nhưng GPU sẽ không hoạt động cho đến khi driver host hoạt động."
    fi


    # --------------------------------------------------------
    # Nếu toolkit chưa có thì cài
    # --------------------------------------------------------

    if command -v nvidia-ctk &>/dev/null; then

        log "NVIDIA Container Toolkit đã có — bỏ qua cài đặt package."

    else

        log "Thêm NVIDIA Container Toolkit repository..."

        install -m 0755 -d /etc/apt/keyrings

        curl -fsSL "${NVIDIA_GPG_URL}" \
            | gpg --dearmor \
                -o /etc/apt/keyrings/nvidia-container-toolkit.gpg

        chmod a+r \
            /etc/apt/keyrings/nvidia-container-toolkit.gpg


        curl -sL "${NVIDIA_LIST_URL}" \
            | sed \
                's#deb https://#deb [signed-by=/etc/apt/keyrings/nvidia-container-toolkit.gpg] https://#g' \
            > /etc/apt/sources.list.d/nvidia-container-toolkit.list


        log "Cài NVIDIA Container Toolkit..."

        apt-get update -qq

        apt-get install -y --no-install-recommends \
            nvidia-container-toolkit
    fi


    # --------------------------------------------------------
    # Configure Docker runtime
    # --------------------------------------------------------

    log "Cấu hình NVIDIA runtime cho Docker..."

    nvidia-ctk runtime configure --runtime=docker
}


# ============================================================
# Docker service
# ============================================================

configure_docker_service() {

    log "Bật Docker khi boot..."

    systemctl enable docker

    log "Khởi động/restart Docker..."

    systemctl restart docker
}


# ============================================================
# Docker group
# ============================================================

add_user_to_docker_group() {

    if ! getent group docker &>/dev/null; then
        log "Tạo group docker..."
        groupadd docker
    fi


    if user_in_docker_group; then

        log "User '${TARGET_USER}' đã thuộc group docker."

    else

        log "Thêm '${TARGET_USER}' vào group docker..."

        usermod -aG docker "${TARGET_USER}"

        echo
        echo "User '${TARGET_USER}' cần đăng xuất/đăng nhập lại"
        echo "hoặc chạy:"
        echo
        echo "  newgrp docker"
    fi
}


# ============================================================
# Verification
# ============================================================

verify_installation() {

    log "Kiểm tra Docker..."

    docker --version

    echo
    docker compose version

    echo
    docker buildx version


    # --------------------------------------------------------
    # hello-world
    # --------------------------------------------------------

    if [[ "${DOCKER_TEST}" == "1" ]]; then

        log "Chạy Docker hello-world test..."

        docker run --rm hello-world

    else

        log "DOCKER_TEST=${DOCKER_TEST} — bỏ qua hello-world test."

    fi


    # --------------------------------------------------------
    # NVIDIA GPU test
    # --------------------------------------------------------

    if [[ "${INSTALL_NVIDIA}" == "1" && "${GPU_TEST}" == "1" ]]; then

        log "Chạy NVIDIA GPU container test..."

        docker run \
            --rm \
            --runtime=nvidia \
            --gpus all \
            ubuntu:22.04 \
            nvidia-smi

    elif [[ "${INSTALL_NVIDIA}" == "1" ]]; then

        log "GPU_TEST=${GPU_TEST} — bỏ qua GPU container test."

    fi


    # --------------------------------------------------------
    # Docker service status
    # --------------------------------------------------------

    log "Trạng thái Docker service..."

    systemctl \
        --no-pager \
        --full \
        status docker \
        | head -n 12 \
        || true
}


# ============================================================
# Main
# ============================================================

install_docker_engine

install_x11_utils

install_nvidia_toolkit

configure_docker_service

add_user_to_docker_group

verify_installation


# ============================================================
# Done
# ============================================================

echo
echo "=========================================="
echo " Hoàn tất Docker Workstation Setup"
echo "=========================================="

echo
echo "User:"
echo "  ${TARGET_USER}"

echo
echo "Bước tiếp theo:"

echo
echo "1. Kích hoạt group docker:"
echo
echo "   newgrp docker"

echo
echo "   hoặc đăng xuất Ubuntu rồi đăng nhập lại."

echo
echo "2. Kiểm tra Docker không cần sudo:"
echo
echo "   docker run --rm hello-world"

if [[ "${INSTALL_NVIDIA}" == "1" ]]; then
    echo
    echo "3. Kiểm tra GPU:"
    echo
    echo "   docker run --rm --runtime=nvidia --gpus all ubuntu:22.04 nvidia-smi"
fi

echo
echo "4. Đăng nhập GitLab Container Registry:"
echo
echo "   docker login registry.gitlab.com"

echo
echo "5. Chạy Docker Compose:"
echo
echo "   cd docker"
echo "   docker compose pull"
echo "   docker compose up -d"

echo
echo "6. Cho phép container truy cập X11:"
echo
echo "   xhost +local:docker"

echo
