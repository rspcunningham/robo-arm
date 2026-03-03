#!/usr/bin/env bash
set -euo pipefail

SERVICE_NAME="${SERVICE_NAME:-robo-arm.service}"
SERVICE_USER="${SERVICE_USER:-pi}"
REPO_ROOT="${REPO_ROOT:-/home/${SERVICE_USER}/robo-arm}"

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
TEMPLATE="${PROJECT_ROOT}/systemd/robo-arm.service"
TMP_FILE="$(mktemp)"

sed \
  -e "s|__USER__|${SERVICE_USER}|g" \
  -e "s|__REPO_ROOT__|${REPO_ROOT}|g" \
  "${TEMPLATE}" > "${TMP_FILE}"

sudo cp "${TMP_FILE}" "/etc/systemd/system/${SERVICE_NAME}"
rm -f "${TMP_FILE}"

sudo systemctl daemon-reload
sudo systemctl enable "${SERVICE_NAME}"
sudo systemctl restart "${SERVICE_NAME}"

echo "[ok] Installed ${SERVICE_NAME}"
