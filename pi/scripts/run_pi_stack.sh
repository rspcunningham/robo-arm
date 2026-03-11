#!/usr/bin/env bash
set -euo pipefail

CLEANUP_TIMEOUT_SEC=2

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
ROS_SETUP_BASH=""

if [[ -z "${ROS_SETUP_BASH}" ]]; then
  if [[ -n "${ROS_DISTRO:-}" && -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]]; then
    ROS_SETUP_BASH="/opt/ros/${ROS_DISTRO}/setup.bash"
  else
    shopt -s nullglob
    ros_setup_candidates=(/opt/ros/*/setup.bash)
    shopt -u nullglob

    if [[ "${#ros_setup_candidates[@]}" -eq 1 ]]; then
      ROS_SETUP_BASH="${ros_setup_candidates[0]}"
    elif [[ "${#ros_setup_candidates[@]}" -gt 1 ]]; then
      ROS_SETUP_BASH="${ros_setup_candidates[0]}"
      echo "[*] Using ROS setup: ${ROS_SETUP_BASH}" >&2
    else
      echo "[err] Could not find a ROS setup script under /opt/ros/*/setup.bash" >&2
      exit 1
    fi
  fi
fi

# ROS setup scripts read optional variables that may be unset, so source them
# with nounset temporarily disabled.
set +u
source "${ROS_SETUP_BASH}"
set -u

cd "${PROJECT_ROOT}"
VENV_BIN="${PROJECT_ROOT}/.venv/bin"
FOLLOWER_ARM_DEVICE="/dev/serial/by-id/usb-Silicon_Labs_CP2102N_USB_to_UART_Bridge_Controller_acff0192339bef11b2e7b69061ce3355-if00-port0"
LEADER_ARM_DEVICE="/dev/serial/by-id/usb-Silicon_Labs_CP2102N_USB_to_UART_Bridge_Controller_24ee9dd6f400f0119b83c3295c2a50c9-if00-port0"

if [[ ! -x "${VENV_BIN}/arm" || ! -x "${VENV_BIN}/cam0" || ! -x "${VENV_BIN}/control-manager" || ! -x "${VENV_BIN}/policy-client" || ! -x "${VENV_BIN}/teleop" || ! -x "${VENV_BIN}/record-manager" || ! -x "${VENV_BIN}/monitor" ]]; then
  echo "[err] Missing venv entrypoints under ${VENV_BIN}. Run 'uv sync --project pi' first." >&2
  exit 1
fi

pids=()

cleanup() {
  local pid
  local attempt
  local remaining
  local max_attempts=$(( CLEANUP_TIMEOUT_SEC * 10 ))

  for pid in "${pids[@]:-}"; do
    if kill -0 "${pid}" >/dev/null 2>&1; then
      kill "${pid}" >/dev/null 2>&1 || true
    fi
  done

  for ((attempt = 0; attempt < max_attempts; attempt++)); do
    remaining=0
    for pid in "${pids[@]:-}"; do
      if kill -0 "${pid}" >/dev/null 2>&1; then
        remaining=1
        break
      fi
    done
    if [[ "${remaining}" == "0" ]]; then
      wait "${pids[@]:-}" 2>/dev/null || true
      return
    fi
    sleep 0.1
  done

  for pid in "${pids[@]:-}"; do
    if kill -0 "${pid}" >/dev/null 2>&1; then
      kill -9 "${pid}" >/dev/null 2>&1 || true
    fi
  done

  wait "${pids[@]:-}" 2>/dev/null || true
}

trap cleanup EXIT INT TERM

"${VENV_BIN}/arm" \
  --ros-args \
  -r __node:=follower_arm \
  -p label:=follower \
  -p "serial_device:=${FOLLOWER_ARM_DEVICE}" &
pids+=("$!")

"${VENV_BIN}/arm" \
  --ros-args \
  -r __node:=leader_arm \
  -p label:=leader \
  -p "serial_device:=${LEADER_ARM_DEVICE}" \
  -p interface_namespace:=/leader &
pids+=("$!")

"${VENV_BIN}/teleop" &
pids+=("$!")

"${VENV_BIN}/record-manager" &
pids+=("$!")

"${VENV_BIN}/cam0" &
pids+=("$!")

"${VENV_BIN}/control-manager" &
pids+=("$!")

"${VENV_BIN}/policy-client" &
pids+=("$!")

"${VENV_BIN}/monitor" &
pids+=("$!")

wait -n "${pids[@]}"
