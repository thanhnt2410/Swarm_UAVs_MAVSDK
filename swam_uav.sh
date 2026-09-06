#!/bin/bash
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
IMAGE="${SW_UAV_IMAGE:-sw-uav:latest}"
NAME="${SW_UAV_CONTAINER:-sw-uav}"
DOCKER_CMD="${SW_UAV_DOCKER_CMD:-docker}"
BUILD_SCRIPT="${SCRIPT_DIR}/docker/build.sh"

# Detect if running inside Docker
IN_DOCKER=0
if [ -f /.dockerenv ] || [ "${SW_UAV_DOCKER:-0}" = "1" ] || [ "${SW_UAV_IN_DOCKER:-0}" = "1" ]; then
    IN_DOCKER=1
fi

# Detect terminal emulator
detect_terminal() {
    if [ "${IN_DOCKER}" -eq 1 ]; then
        if command -v xfce4-terminal >/dev/null 2>&1; then
            echo "xfce4-terminal"
        elif command -v xterm >/dev/null 2>&1; then
            echo "xterm"
        elif command -v gnome-terminal >/dev/null 2>&1; then
            echo "gnome-terminal"
        else
            echo "none"
        fi
    else
        if command -v gnome-terminal >/dev/null 2>&1; then
            echo "gnome-terminal"
        elif command -v xfce4-terminal >/dev/null 2>&1; then
            echo "xfce4-terminal"
        elif command -v xterm >/dev/null 2>&1; then
            echo "xterm"
        else
            echo "none"
        fi
    fi
}

TERM_TYPE="$(detect_terminal)"
echo "[sw-uav] Running environment: $([ "${IN_DOCKER}" -eq 1 ] && echo 'Inside Docker' || echo 'Host machine')"
echo "[sw-uav] Terminal emulator: ${TERM_TYPE}"

open_terminal_window() {
    local title="$1"
    local cmd="$2"

    case "${TERM_TYPE}" in
        "xfce4-terminal")
            xfce4-terminal --title="${title}" -x bash -c "${cmd}" &
            ;;
        "xterm")
            xterm -T "${title}" -e bash -c "${cmd}" &
            ;;
        "gnome-terminal")
            gnome-terminal --title="${title}" -- bash -c "${cmd}" &
            ;;
        *)
            echo "[sw-uav] Warning: No terminal emulator found. Running in background: ${title}"
            bash -c "${cmd}" &
            ;;
    esac
}

if [ "${IN_DOCKER}" -eq 0 ]; then
    # -------------------------------------------------------------
    # HOST ENVIRONMENT SETUP
    # -------------------------------------------------------------
    echo "[sw-uav] Checking Docker access..."
    if ! ${DOCKER_CMD} ps >/dev/null 2>&1; then
        echo "[sw-uav] Docker needs sudo. You may be asked for your password."
        sudo -v
        if sudo docker ps >/dev/null 2>&1; then
            DOCKER_CMD="sudo docker"
        else
            echo "Cannot access Docker. Start Docker or set SW_UAV_DOCKER_CMD."
            exit 1
        fi
    fi

    echo "[sw-uav] Using Docker command: ${DOCKER_CMD}"
    xhost +local:docker >/dev/null 2>&1 || true
    xhost +local:root >/dev/null 2>&1 || true

    TORCH_VARIANT="$(SW_UAV_DOCKER_CMD="${DOCKER_CMD}" "${BUILD_SCRIPT}" --detect)"
    IMAGE_TORCH_VARIANT="$(${DOCKER_CMD} image inspect \
        --format '{{index .Config.Labels "org.sw-uav.torch-variant"}}' "${IMAGE}" 2>/dev/null || true)"

    if [ "${IMAGE_TORCH_VARIANT}" != "${TORCH_VARIANT}" ]; then
        echo "[sw-uav] Image '${IMAGE}' is missing or uses '${IMAGE_TORCH_VARIANT:-unknown}', expected '${TORCH_VARIANT}'."
        SW_UAV_DOCKER_CMD="${DOCKER_CMD}" SW_UAV_TORCH_VARIANT="${TORCH_VARIANT}" "${BUILD_SCRIPT}"
    fi

    DOCKER_ACCELERATOR_ARGS=()
    DOCKER_DEVICE_ARGS=()
    if [ "${TORCH_VARIANT}" = "cpu" ]; then
        echo "[sw-uav] Accelerator: CPU PyTorch; host AMD/Mesa graphics are supported."
    else
        echo "[sw-uav] Accelerator: NVIDIA CUDA (${TORCH_VARIANT})."
        DOCKER_ACCELERATOR_ARGS=(
            --gpus all
            -e __GLX_VENDOR_LIBRARY_NAME=nvidia
            -e NVIDIA_VISIBLE_DEVICES=all
            -e "NVIDIA_DRIVER_CAPABILITIES=compute,utility,graphics,video,display"
        )
    fi
    if [ -d /dev/dri ]; then
        DOCKER_DEVICE_ARGS=(--device /dev/dri:/dev/dri)
    fi

    if ${DOCKER_CMD} ps -a --format '{{.Names}}' | grep -Fxq "${NAME}"; then
        CURRENT_IMAGE_ID="$(${DOCKER_CMD} image inspect --format '{{.Id}}' "${IMAGE}")"
        CONTAINER_IMAGE_ID="$(${DOCKER_CMD} inspect --format '{{.Image}}' "${NAME}")"
        if [ "${CURRENT_IMAGE_ID}" != "${CONTAINER_IMAGE_ID}" ]; then
            echo "[sw-uav] Recreating '${NAME}' because its image or accelerator variant changed."
            ${DOCKER_CMD} rm -f "${NAME}" >/dev/null
        fi
    fi

    if ! ${DOCKER_CMD} ps -a --format '{{.Names}}' | grep -Fxq "${NAME}"; then
        echo "[sw-uav] Container '${NAME}' not found. Creating it from image '${IMAGE}'..."
        ${DOCKER_CMD} run -dit \
            "${DOCKER_ACCELERATOR_ARGS[@]}" \
            "${DOCKER_DEVICE_ARGS[@]}" \
            --net=host \
            --ipc=host \
            -e SW_UAV_DOCKER=1 \
            -e DISPLAY="${DISPLAY:-:0}" \
            -e QT_X11_NO_MITSHM=1 \
            -e QT_XCB_GL_INTEGRATION=xcb_glx \
            -e SW_UAV_TORCH_VARIANT="${TORCH_VARIANT}" \
            -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
            -v "${SCRIPT_DIR}:/app" \
            -w /app \
            --name "${NAME}" \
            "${IMAGE}" \
            zsh >/dev/null
    elif [ "$(${DOCKER_CMD} inspect -f '{{.State.Running}}' "${NAME}")" != "true" ]; then
        echo "[sw-uav] Starting existing container '${NAME}'..."
        ${DOCKER_CMD} start "${NAME}" >/dev/null
    else
        echo "[sw-uav] Container '${NAME}' is already running."
    fi

    echo "[sw-uav] Preparing PX4 workspace inside Docker..."
    ${DOCKER_CMD} exec "${NAME}" git config --global --add safe.directory '*' || true

    ${DOCKER_CMD} exec "${NAME}" zsh -lc "
        pkill -x px4 || true
        pkill -x gz || true
        pkill -x gazebo || true
        pkill -x ruby || true
        python3 /app/px4_extensions/battery_power/install.py \
            --px4-dir /app/dependencies/PX4-Autopilot \
            --config /app/config/uav_config.yaml
    "

    echo "[sw-uav] PX4 workspace is ready."
    echo "[sw-uav] Opening PX4 main terminal..."
    open_terminal_window "PX4 main" "${DOCKER_CMD} exec -it ${NAME} zsh -lc 'cd /app/dependencies/PX4-Autopilot && make px4_sitl gz_x500; exec zsh'"
    sleep 10

    for i in {1..5}
    do
        pose="$((i*3)),0,0.3,0,0,0"
        echo "[sw-uav] Opening PX4 UAV ${i} terminal..."
        open_terminal_window "PX4 UAV ${i}" "${DOCKER_CMD} exec -it ${NAME} zsh -lc 'cd /app/dependencies/PX4-Autopilot && PX4_GZ_STANDALONE=1 PX4_SYS_AUTOSTART=4001 PX4_GZ_MODEL=x500 PX4_GZ_MODEL_POSE=\"${pose}\" ./build/px4_sitl_default/bin/px4 -i ${i}; exec zsh'"
        sleep 1
    done

else
    # -------------------------------------------------------------
    # INSIDE DOCKER CONTAINER SETUP
    # -------------------------------------------------------------
    echo "[sw-uav] Preparing PX4 workspace..."
    git config --global --add safe.directory '*' || true

    pkill -x px4 || true
    pkill -x gz || true
    pkill -x gazebo || true
    pkill -x ruby || true
    python3 /app/px4_extensions/battery_power/install.py \
        --px4-dir /app/dependencies/PX4-Autopilot \
        --config /app/config/uav_config.yaml

    echo "[sw-uav] PX4 workspace is ready."
    echo "[sw-uav] Opening PX4 main terminal..."
    open_terminal_window "PX4 main" "cd /app/dependencies/PX4-Autopilot && make px4_sitl gz_x500; exec zsh"
    sleep 10

    for i in {1..5}
    do
        pose="$((i*3)),0,0.3,0,0,0"
        echo "[sw-uav] Opening PX4 UAV ${i} terminal..."
        open_terminal_window "PX4 UAV ${i}" "cd /app/dependencies/PX4-Autopilot && PX4_GZ_STANDALONE=1 PX4_SYS_AUTOSTART=4001 PX4_GZ_MODEL=x500 PX4_GZ_MODEL_POSE=\"${pose}\" ./build/px4_sitl_default/bin/px4 -i ${i}; exec zsh"
        sleep 1
    done
fi

echo "[sw-uav] Done. PX4 terminals were launched."
