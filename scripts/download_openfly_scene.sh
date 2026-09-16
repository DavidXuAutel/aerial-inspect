#!/usr/bin/env bash
# Download OpenFly AirSim scene zip from HuggingFace (gated — set HF_TOKEN if needed).
#
# Usage:
#   bash scripts/download_openfly_scene.sh env_airsim_gz
#   HF_TOKEN=hf_... bash scripts/download_openfly_scene.sh env_airsim_gz
set -euo pipefail

SCENE_ID="${1:-${AIRSIM_SCENE_ID:-env_airsim_gz}}"
DEST="${AIRSIM_PERSISTENT:-$HOME/aerial_airsim_persistent}/scene/${SCENE_ID}"
ZIP_NAME="${SCENE_ID}.zip"
HF_REPO="IPEC-COMMUNITY/OpenFly_DataGen"
HF_PATH="airsim/${ZIP_NAME}"

mkdir -p "$DEST"

if [[ -x "$DEST/LinuxNoEditor/start.sh" ]]; then
  echo "[download_openfly] already present: $DEST/LinuxNoEditor/start.sh"
  exit 0
fi

TMP="${TMPDIR:-/tmp}/openfly_${SCENE_ID}_$$"
mkdir -p "$TMP"
trap 'rm -rf "$TMP"' EXIT

PY="${AERIAL_PY:-${PYTHON_BIN:-}}"
if [[ -z "$PY" || ! -x "$PY" ]]; then
  PY="$(command -v python3 || true)"
fi

echo "[download_openfly] scene=$SCENE_ID -> $DEST"

if command -v hf >/dev/null 2>&1; then
  hf download "$HF_REPO" "$HF_PATH" --local-dir "$TMP"
  ZIP="$TMP/$HF_PATH"
elif command -v huggingface-cli >/dev/null 2>&1; then
  huggingface-cli download "$HF_REPO" "$HF_PATH" --local-dir "$TMP" --local-dir-use-symlinks False
  ZIP="$TMP/$HF_PATH"
else
  "$PY" - <<PY
import os, sys
from pathlib import Path
try:
    from huggingface_hub import hf_hub_download
except ImportError:
    print("Install: pip install huggingface_hub", file=sys.stderr)
    sys.exit(1)
token = os.environ.get("HF_TOKEN") or os.environ.get("HUGGINGFACE_HUB_TOKEN")
path = hf_hub_download(
    repo_id="${HF_REPO}",
    filename="${HF_PATH}",
    token=token,
    local_dir="${TMP}",
)
print(path)
PY
  ZIP="$(find "$TMP" -name "$ZIP_NAME" | head -1)"
fi

if [[ ! -f "$ZIP" ]]; then
  echo "[download_openfly] zip not found under $TMP" >&2
  echo "  Accept dataset terms: https://huggingface.co/datasets/${HF_REPO}" >&2
  echo "  Then: HF_TOKEN=hf_... bash $0 $SCENE_ID" >&2
  exit 1
fi

echo "[download_openfly] unzip $(du -h "$ZIP" | awk '{print $1}') ..."
unzip -q "$ZIP" -d "$DEST"

if [[ ! -x "$DEST/LinuxNoEditor/start.sh" ]]; then
  # Some zips nest one directory level.
  nested="$(find "$DEST" -path '*/LinuxNoEditor/start.sh' | head -1)"
  if [[ -n "$nested" ]]; then
    root="$(dirname "$(dirname "$nested")")"
    if [[ "$root" != "$DEST" ]]; then
      shopt -s dotglob
      mv "$root"/* "$DEST"/
      rmdir "$root" 2>/dev/null || true
    fi
  fi
fi

if [[ ! -x "$DEST/LinuxNoEditor/start.sh" ]]; then
  echo "[download_openfly] start.sh missing after unzip" >&2
  find "$DEST" -maxdepth 3 -type f | head -20 >&2
  exit 1
fi

echo "[download_openfly] ok: $DEST/LinuxNoEditor/start.sh"
