#!/usr/bin/env bash
set -euo pipefail

PI_HOST="${PI_HOST:-pi@pi1.local}"
PI_DIR="${PI_DIR:-/home/pi/robo-arm}"
ROBO_ARM_POLICY_URL="${ROBO_ARM_POLICY_URL:-http://Robins-MacBook-Pro.local:8000/predict}"
RESTART_SERVICE="${RESTART_SERVICE:-1}"
VERIFY_SERVICE="${VERIFY_SERVICE:-1}"
SYNC_SERVICE_UNIT="${SYNC_SERVICE_UNIT:-1}"
LOG_LINES="${LOG_LINES:-30}"
RESTART_SINCE_FILE="${RESTART_SINCE_FILE:-/tmp/robo-arm-restart-since}"
SERVICE_DROPIN_DIR="/etc/systemd/system/robo-arm.service.d"
SERVICE_DROPIN_FILE="${SERVICE_DROPIN_DIR}/10-runtime-env.conf"

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
UNIT_EXISTS_CMD="test -e /etc/systemd/system/robo-arm.service"

echo "[*] Syncing repo to ${PI_HOST}:${PI_DIR}"
ssh "${PI_HOST}" "mkdir -p '${PI_DIR}'"
rsync -az --delete \
  --exclude '.venv/' \
  --exclude '__pycache__/' \
  --exclude '*/.pio/' \
  --exclude 'data/' \
  --exclude 'log/' \
  "${REPO_ROOT}/pi/" "${PI_HOST}:${PI_DIR}/pi/"

runtime_env_lines=()
for env_name in ROBO_ARM_POLICY_URL ROBO_ARM_POLICY_RATE_HZ ROBO_ARM_POLICY_TIMEOUT_SEC ROBO_ARM_MONITOR_PORT; do
  env_value="${!env_name-}"
  if [[ -z "${env_value}" ]]; then
    continue
  fi
  escaped_value="${env_value//\\/\\\\}"
  escaped_value="${escaped_value//\"/\\\"}"
  runtime_env_lines+=("Environment=\"${env_name}=${escaped_value}\"")
done

if [[ "${#runtime_env_lines[@]}" -gt 0 ]]; then
  echo "[*] Updating robo-arm.service runtime environment"
  tmp_env_file="$(mktemp)"
  {
    echo "[Service]"
    printf '%s\n' "${runtime_env_lines[@]}"
  } > "${tmp_env_file}"
  scp "${tmp_env_file}" "${PI_HOST}:/tmp/robo-arm-runtime-env.conf" >/dev/null
  rm -f "${tmp_env_file}"
  ssh "${PI_HOST}" "sudo mkdir -p '${SERVICE_DROPIN_DIR}' && sudo mv /tmp/robo-arm-runtime-env.conf '${SERVICE_DROPIN_FILE}'"
fi

echo "[*] Installing Pi dependencies"
if ! ssh "${PI_HOST}" "bash -lc 'cd \"${PI_DIR}\" && uv sync --project pi --locked'"; then
  echo "[error] Pi dependency sync failed. Not restarting robo-arm.service; the currently running service was left unchanged." >&2
  exit 1
fi

if [[ "${RESTART_SERVICE}" == "1" ]]; then
  if [[ "${SYNC_SERVICE_UNIT}" == "1" ]]; then
    echo "[*] Refreshing robo-arm.service unit and restarting"
    ssh "${PI_HOST}" "bash -lc 'if ${UNIT_EXISTS_CMD}; then date --iso-8601=seconds > \"${RESTART_SINCE_FILE}\" && cd \"${PI_DIR}/pi\" && ./scripts/install_pi_service.sh >/dev/null; else echo \"[skip] robo-arm.service not installed\"; fi'"
  else
    echo "[*] Restarting robo-arm.service if installed"
    ssh "${PI_HOST}" "bash -lc 'if ${UNIT_EXISTS_CMD}; then date --iso-8601=seconds > \"${RESTART_SINCE_FILE}\" && sudo systemctl restart robo-arm.service; else echo \"[skip] robo-arm.service not installed\"; fi'"
  fi
fi

if [[ "${VERIFY_SERVICE}" == "1" ]]; then
  echo "[*] Checking robo-arm.service state"
  ssh "${PI_HOST}" "bash -lc '
    if ${UNIT_EXISTS_CMD}; then
      sudo systemctl is-active robo-arm.service || true
      sudo systemctl status --no-pager robo-arm.service
      echo
      service_since=\"\$(cat \"${RESTART_SINCE_FILE}\" 2>/dev/null || true)\"
      if [[ -n \"\${service_since}\" ]]; then
        echo \"[*] Recent logs (since \${service_since})\"
        sudo journalctl -u robo-arm.service --since \"\${service_since}\" --no-pager
      else
        echo \"[*] Recent logs (last ${LOG_LINES} lines)\"
        sudo journalctl -u robo-arm.service -n ${LOG_LINES} --no-pager
      fi
    else
      echo \"[skip] robo-arm.service not installed\"
    fi
  '"
fi

echo "[ok] Deploy complete"
