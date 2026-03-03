#!/usr/bin/env bash
set -euo pipefail

# codex-update.sh — Install or update OpenAI Codex on aarch64 Linux (e.g. Raspberry Pi 4)
#
# Usage:
#   codex-update           # install/update to latest stable release
#   codex-update --alpha   # install/update to latest prerelease (alpha/beta)

REPO="openai/codex"
ASSET_NAME="codex-aarch64-unknown-linux-gnu.tar.gz"
INSTALL_DIR="/usr/local/bin"
TMP_DIR="$(mktemp -d)"

cleanup() { rm -rf "$TMP_DIR"; }
trap cleanup EXIT

INCLUDE_PRERELEASE=false
if [[ "${1:-}" == "--alpha" ]]; then
  INCLUDE_PRERELEASE=true
fi

# Get release tag from GitHub API
echo "Checking latest release..."
if $INCLUDE_PRERELEASE; then
  # First tag from all releases (sorted newest first by GitHub)
  LATEST_TAG=$(curl -sL "https://api.github.com/repos/${REPO}/releases?per_page=1" \
    | grep -oP '"tag_name"\s*:\s*"\K[^"]+' | head -1)
else
  # "latest" endpoint only returns non-prerelease
  LATEST_TAG=$(curl -sL "https://api.github.com/repos/${REPO}/releases/latest" \
    | grep -oP '"tag_name"\s*:\s*"\K[^"]+')
fi

if [ -z "$LATEST_TAG" ]; then
  echo "Error: Could not fetch latest release tag." >&2
  exit 1
fi

echo "Latest release: ${LATEST_TAG}"

# Check currently installed version (if any)
if command -v codex &>/dev/null; then
  CURRENT=$(codex --version 2>/dev/null || echo "unknown")
  echo "Installed version: ${CURRENT}"
  # Strip "rust-v" prefix for comparison
  TAG_VERSION="${LATEST_TAG#rust-v}"
  if echo "$CURRENT" | grep -qF "$TAG_VERSION"; then
    echo "Already up to date."
    exit 0
  fi
else
  echo "Codex not currently installed."
fi

# Download
DOWNLOAD_URL="https://github.com/${REPO}/releases/download/${LATEST_TAG}/${ASSET_NAME}"
echo "Downloading ${DOWNLOAD_URL}..."
curl -fL --progress-bar -o "${TMP_DIR}/${ASSET_NAME}" "$DOWNLOAD_URL"

# Extract
echo "Extracting..."
tar xzf "${TMP_DIR}/${ASSET_NAME}" -C "$TMP_DIR"

# The archive contains a single binary named like "codex-aarch64-unknown-linux-gnu"
# Find it and install as "codex"
BINARY=$(find "$TMP_DIR" -maxdepth 1 -type f -name "codex*" ! -name "*.gz" ! -name "*.tar" | head -1)
if [ -z "$BINARY" ]; then
  echo "Error: Could not find codex binary in archive." >&2
  echo "Archive contents:"
  ls -la "$TMP_DIR"
  exit 1
fi

chmod +x "$BINARY"
sudo mv "$BINARY" "${INSTALL_DIR}/codex"
echo "Installed codex to ${INSTALL_DIR}/codex"

# Verify
echo ""
codex --version
echo "Done."
