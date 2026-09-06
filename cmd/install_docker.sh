#!/usr/bin/env bash

set -Eeuo pipefail

# ============================================================
# Cài Docker Engine trên Ubuntu 22.04
# Bao gồm:
#   - Docker Engine
#   - Docker CLI
#   - containerd
#   - Docker Buildx
#   - Docker Compose Plugin
#   - Thêm user hiện tại vào nhóm docker
# ============================================================

if [[ "${EUID}" -eq 0 ]]; then
  echo "Không chạy script bằng sudo hoặc tài khoản root."
  echo "Hãy chạy: ./install_docker.sh"
  exit 1
fi

if [[ ! -f /etc/os-release ]]; then
  echo "Không thể xác định hệ điều hành."
  exit 1
fi

# shellcheck disable=SC1091
source /etc/os-release

if [[ "${ID}" != "ubuntu" ]]; then
  echo "Script này chỉ dành cho Ubuntu."
  echo "Hệ điều hành hiện tại: ${PRETTY_NAME:-không xác định}"
  exit 1
fi

if [[ "${VERSION_ID}" != "22.04" ]]; then
  echo "Cảnh báo: script được thiết kế cho Ubuntu 22.04."
  echo "Phiên bản hiện tại: ${VERSION_ID}"
  read -r -p "Bạn vẫn muốn tiếp tục? [y/N]: " answer

  if [[ ! "${answer}" =~ ^[Yy]$ ]]; then
    exit 1
  fi
fi

echo "=========================================="
echo " Bắt đầu cài đặt Docker Engine"
echo " Hệ điều hành: ${PRETTY_NAME}"
echo " Kiến trúc: $(dpkg --print-architecture)"
echo " User: ${USER}"
echo "=========================================="

echo
echo "[1/8] Gỡ các package Docker có thể gây xung đột..."

conflicting_packages=(
  docker.io
  docker-doc
  docker-compose
  docker-compose-v2
  podman-docker
  containerd
  runc
)

for package in "${conflicting_packages[@]}"; do
  if dpkg -s "${package}" >/dev/null 2>&1; then
    sudo apt-get remove -y "${package}"
  fi
done

echo
echo "[2/8] Cập nhật package và cài công cụ cần thiết..."

sudo apt-get update

sudo apt-get install -y \
  ca-certificates \
  curl

echo
echo "[3/8] Thêm Docker GPG key..."

sudo install -m 0755 -d /etc/apt/keyrings

sudo curl -fsSL \
  https://download.docker.com/linux/ubuntu/gpg \
  -o /etc/apt/keyrings/docker.asc

sudo chmod a+r /etc/apt/keyrings/docker.asc

echo
echo "[4/8] Thêm Docker APT repository..."

ARCHITECTURE="$(dpkg --print-architecture)"
UBUNTU_CODENAME="${UBUNTU_CODENAME:-${VERSION_CODENAME}}"

echo \
  "deb [arch=${ARCHITECTURE} signed-by=/etc/apt/keyrings/docker.asc] https://download.docker.com/linux/ubuntu ${UBUNTU_CODENAME} stable" \
  | sudo tee /etc/apt/sources.list.d/docker.list >/dev/null

echo
echo "[5/8] Cài Docker Engine và Docker Compose..."

sudo apt-get update

sudo apt-get install -y \
  docker-ce \
  docker-ce-cli \
  containerd.io \
  docker-buildx-plugin \
  docker-compose-plugin

echo
echo "[6/8] Khởi động và bật Docker tự động..."

sudo systemctl enable --now docker

echo
echo "[7/8] Thêm user '${USER}' vào nhóm docker..."

if ! getent group docker >/dev/null; then
  sudo groupadd docker
fi

sudo usermod -aG docker "${USER}"

echo
echo "[8/8] Kiểm tra cài đặt..."

sudo docker run --rm hello-world

echo
echo "=========================================="
echo " Cài đặt Docker thành công"
echo "=========================================="

echo
echo "Docker:"
sudo docker --version

echo
echo "Docker Compose:"
sudo docker compose version

echo
echo "Docker Buildx:"
sudo docker buildx version

echo
echo "Trạng thái Docker:"
sudo systemctl --no-pager --full status docker | head -n 12 || true

echo
echo "User '${USER}' đã được thêm vào nhóm docker."
echo
echo "Để dùng Docker không cần sudo, thực hiện một trong hai cách:"
echo
echo "  Cách 1: đăng xuất Ubuntu rồi đăng nhập lại."
echo
echo "  Cách 2: chạy ngay:"
echo
echo "    newgrp docker"
echo
echo "Sau đó kiểm tra:"
echo
echo "    docker run --rm hello-world"
echo "    docker compose version"
