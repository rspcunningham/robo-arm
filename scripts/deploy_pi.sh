#!/usr/bin/env bash
set -euo pipefail

PI_HOST="${PI_HOST:-pi@pi1.local}"
PI_DIR="${PI_DIR:-/home/pi/robo-arm}"
RESTART_SERVICE="${RESTART_SERVICE:-1}"
VERIFY_SERVICE="${VERIFY_SERVICE:-1}"
LOG_LINES="${LOG_LINES:-30}"

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
UNIT_EXISTS_CMD="test -e /etc/systemd/system/robo-arm.service"

echo "[*] Syncing repo to ${PI_HOST}:${PI_DIR}"
ssh "${PI_HOST}" "mkdir -p '${PI_DIR}'"
rsync -az --delete \
  --exclude '.git/' \
  --exclude '.venv/' \
  --exclude 'pi/.venv/' \
  --exclude 'experiments/.venv/' \
  --exclude '__pycache__/' \
  --exclude '*/.pio/' \
  --exclude 'pi/data/' \
  --exclude 'pi/log/' \
  "${REPO_ROOT}/" "${PI_HOST}:${PI_DIR}/"

echo "[*] Installing Pi dependencies"
ssh "${PI_HOST}" "bash -lc 'cd \"${PI_DIR}\" && uv sync --project pi --frozen'"

if [[ "${RESTART_SERVICE}" == "1" ]]; then
  echo "[*] Restarting robo-arm.service if installed"
  ssh "${PI_HOST}" "bash -lc 'if ${UNIT_EXISTS_CMD}; then sudo systemctl restart robo-arm.service; else echo \"[skip] robo-arm.service not installed\"; fi'"
fi

if [[ "${VERIFY_SERVICE}" == "1" ]]; then
  echo "[*] Checking robo-arm.service state"
  ssh "${PI_HOST}" "bash -lc '
    if ${UNIT_EXISTS_CMD}; then
      sudo systemctl is-active robo-arm.service || true
      sudo systemctl status --no-pager robo-arm.service
      echo
      echo \"[*] Recent logs (last ${LOG_LINES} lines)\"
      sudo journalctl -u robo-arm.service -n ${LOG_LINES} --no-pager
    else
      echo \"[skip] robo-arm.service not installed\"
    fi
  '"
fi

echo "[ok] Deploy complete"
