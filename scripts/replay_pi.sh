#!/usr/bin/env bash
set -euo pipefail

PI_HOST="${PI_HOST:-pi@pi1.local}"
PI_DIR="${PI_DIR:-/home/pi/robo-arm}"

if [[ "$#" -eq 0 ]]; then
  echo "Usage: $0 --repo-id <repo> [replay args...]" >&2
  exit 1
fi

args=()
for arg in "$@"; do
  args+=("$(printf '%q' "${arg}")")
done

remote_cmd="source /opt/ros/*/setup.bash && cd $(printf '%q' "${PI_DIR}/pi") && ./.venv/bin/replay ${args[*]}"
ssh -t "${PI_HOST}" "bash -lc $(printf '%q' "${remote_cmd}")"
