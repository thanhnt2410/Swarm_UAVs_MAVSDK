#!/usr/bin/env bash
set -Eeuo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
IMAGE="${SW_UAV_IMAGE:-sw-uav:latest}"
REQUESTED_VARIANT="${SW_UAV_TORCH_VARIANT:-auto}"

read -r -a DOCKER_COMMAND <<< "${SW_UAV_DOCKER_CMD:-docker}"

docker_command() {
    "${DOCKER_COMMAND[@]}" "$@"
}

log() {
    echo "[sw-uav] $*" >&2
}

version_at_least() {
    local actual="$1"
    local required="$2"
    [[ "$(printf '%s\n' "${required}" "${actual}" | sort -V | head -n 1)" == "${required}" ]]
}

docker_has_nvidia_runtime() {
    docker_command info --format '{{json .Runtimes}}' 2>/dev/null | grep -q '"nvidia"'
}

detect_torch_variant() {
    case "${REQUESTED_VARIANT}" in
        cpu|cu118|cu124|cu126)
            log "Using requested PyTorch variant: ${REQUESTED_VARIANT}"
            echo "${REQUESTED_VARIANT}"
            return 0
            ;;
        auto) ;;
        *)
            echo "SW_UAV_TORCH_VARIANT must be auto, cpu, cu118, cu124, or cu126." >&2
            return 2
            ;;
    esac

    if ! command -v nvidia-smi >/dev/null 2>&1 || ! nvidia-smi -L >/dev/null 2>&1; then
        log "No working NVIDIA GPU detected; selecting CPU PyTorch (AMD-compatible)."
        echo "cpu"
        return 0
    fi

    if ! docker_has_nvidia_runtime; then
        log "NVIDIA GPU found, but the NVIDIA Docker runtime is unavailable; selecting CPU PyTorch."
        echo "cpu"
        return 0
    fi

    local cuda_version
    cuda_version="$(nvidia-smi | sed -n 's/.*CUDA Version: \([0-9][0-9.]*\).*/\1/p' | head -n 1)"

    if [[ -z "${cuda_version}" ]]; then
        log "Could not determine the CUDA level supported by the driver; selecting CPU PyTorch."
        echo "cpu"
    elif version_at_least "${cuda_version}" "12.6"; then
        log "CUDA ${cuda_version} detected; selecting PyTorch cu126."
        echo "cu126"
    elif version_at_least "${cuda_version}" "12.4"; then
        log "CUDA ${cuda_version} detected; selecting PyTorch cu124."
        echo "cu124"
    elif version_at_least "${cuda_version}" "11.8"; then
        log "CUDA ${cuda_version} detected; selecting PyTorch cu118."
        echo "cu118"
    else
        log "CUDA ${cuda_version} is older than the supported wheels; selecting CPU PyTorch."
        echo "cpu"
    fi
}

TORCH_VARIANT="$(detect_torch_variant)"

if [[ "${1:-}" == "--detect" ]]; then
    echo "${TORCH_VARIANT}"
    exit 0
fi

if [[ $# -gt 0 ]]; then
    echo "Usage: $0 [--detect]" >&2
    exit 2
fi

log "Building ${IMAGE} with PyTorch variant ${TORCH_VARIANT}."
docker_command build --progress=plain \
    --build-arg "PYTORCH_VARIANT=${TORCH_VARIANT}" \
    -t "${IMAGE}" \
    -f "${SCRIPT_DIR}/Dockerfile" \
    "${PROJECT_DIR}"
