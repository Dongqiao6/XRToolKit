#!/usr/bin/env bash
# Download the MediaPipe task models used by vision_bridge.
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
MODELS="$ROOT/models"
mkdir -p "$MODELS"

fetch() {
  local url="$1" dest="$2"
  if [ -s "$dest" ]; then
    echo "have   $(basename "$dest") ($(stat -c%s "$dest") bytes)"
    return
  fi
  echo "fetch  $(basename "$dest")"
  curl -fL --retry 3 --retry-delay 2 -o "$dest.part" "$url"
  mv "$dest.part" "$dest"
  echo "ok     $(basename "$dest") ($(stat -c%s "$dest") bytes)"
}

fetch "https://storage.googleapis.com/mediapipe-models/pose_landmarker/pose_landmarker_full/float16/1/pose_landmarker_full.task" \
      "$MODELS/pose_landmarker_full.task"
fetch "https://storage.googleapis.com/mediapipe-models/hand_landmarker/hand_landmarker/float16/1/hand_landmarker.task" \
      "$MODELS/hand_landmarker.task"

sha256sum "$MODELS"/*.task
