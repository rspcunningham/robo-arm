#!/usr/bin/env bash
set -euo pipefail

DISABLE_CAM="${DISABLE_CAM:-0}"
ENABLE_MONITOR="${ENABLE_MONITOR:-1}"
ROBO_ARM_MONITOR_PORT="${ROBO_ARM_MONITOR_PORT:-8080}"
ROS_SETUP_BASH="${ROS_SETUP_BASH:-}"
UV_BIN="${UV_BIN:-}"

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"

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

if [[ -z "${UV_BIN}" ]]; then
  if command -v uv >/dev/null 2>&1; then
    UV_BIN="$(command -v uv)"
  elif [[ -n "${HOME:-}" && -x "${HOME}/.local/bin/uv" ]]; then
    UV_BIN="${HOME}/.local/bin/uv"
  elif [[ -x "/home/pi/.local/bin/uv" ]]; then
    UV_BIN="/home/pi/.local/bin/uv"
  else
    echo "[err] Could not find uv. Set UV_BIN explicitly." >&2
    exit 1
  fi
fi

cd "${PROJECT_ROOT}"

pids=()

cleanup() {
  local pid
  for pid in "${pids[@]:-}"; do
    if kill -0 "${pid}" >/dev/null 2>&1; then
      kill "${pid}" >/dev/null 2>&1 || true
    fi
  done
  wait "${pids[@]:-}" 2>/dev/null || true
}

trap cleanup EXIT INT TERM

"${UV_BIN}" run --project "${PROJECT_ROOT}" arm &
pids+=("$!")

if [[ "${DISABLE_CAM}" != "1" ]]; then
  "${UV_BIN}" run --project "${PROJECT_ROOT}" cam &
  pids+=("$!")
fi

if [[ "${ENABLE_MONITOR}" == "1" ]]; then
  "${UV_BIN}" run --project "${PROJECT_ROOT}" monitor --port "${ROBO_ARM_MONITOR_PORT}" &
  pids+=("$!")
fi

wait -n "${pids[@]}"
