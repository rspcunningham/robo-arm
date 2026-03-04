#!/usr/bin/env bash
set -euo pipefail

ROBO_ARM_MONITOR_PORT="${ROBO_ARM_MONITOR_PORT:-8080}"
ROBO_ARM_POLICY_URL="${ROBO_ARM_POLICY_URL:-http://127.0.0.1:8000/predict}"
ROBO_ARM_POLICY_RATE_HZ="${ROBO_ARM_POLICY_RATE_HZ:-5}"
ROBO_ARM_POLICY_TIMEOUT_SEC="${ROBO_ARM_POLICY_TIMEOUT_SEC:-1}"
ROS_SETUP_BASH="${ROS_SETUP_BASH:-}"
CLEANUP_TIMEOUT_SEC="${CLEANUP_TIMEOUT_SEC:-2}"

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

cd "${PROJECT_ROOT}"
VENV_BIN="${PROJECT_ROOT}/.venv/bin"

if [[ ! -x "${VENV_BIN}/arm" || ! -x "${VENV_BIN}/cam" || ! -x "${VENV_BIN}/control-manager" || ! -x "${VENV_BIN}/policy-client" || ! -x "${VENV_BIN}/monitor" ]]; then
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

"${VENV_BIN}/arm" &
pids+=("$!")

"${VENV_BIN}/cam" &
pids+=("$!")

"${VENV_BIN}/control-manager" &
pids+=("$!")

"${VENV_BIN}/policy-client" \
  --url "${ROBO_ARM_POLICY_URL}" \
  --rate-hz "${ROBO_ARM_POLICY_RATE_HZ}" \
  --timeout-sec "${ROBO_ARM_POLICY_TIMEOUT_SEC}" &
pids+=("$!")

"${VENV_BIN}/monitor" --port "${ROBO_ARM_MONITOR_PORT}" &
pids+=("$!")

wait -n "${pids[@]}"
