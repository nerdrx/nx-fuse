#!/usr/bin/env bash
set -euo pipefail

project_dir="$(cd "$(dirname "$0")" && pwd)"
venv_dir="$project_dir/.venv"
model_dir="$project_dir/artifacts/models"
model_path="$model_dir/pose_landmarker_lite.task"
model_url="https://storage.googleapis.com/mediapipe-models/pose_landmarker/pose_landmarker_lite/float16/1/pose_landmarker_lite.task"
model_sha256="59929e1d1ee95287735ddd833b19cf4ac46d29bc7afddbbf6753c459690d574a"

download_model=0
if [[ $# -eq 1 && "$1" == "--download-model" ]]; then
    download_model=1
elif [[ $# -ne 0 ]]; then
    printf 'usage: %s [--download-model]\n' "$0" >&2
    exit 2
fi

if [[ ! -x "$venv_dir/bin/python" ]]; then
    python3 -m venv "$venv_dir"
fi
"$venv_dir/bin/python" -m pip install -r "$project_dir/requirements-camera.txt"

if (( download_model )); then
    mkdir -p "$model_dir"
    tmp_path="$(mktemp "$model_dir/.pose_landmarker_lite.task.XXXXXX")"
    trap 'rm -f "$tmp_path"' EXIT
    curl --fail --location --output "$tmp_path" "$model_url"
    printf '%s  %s\n' "$model_sha256" "$tmp_path" | sha256sum --check --status
    mv -f "$tmp_path" "$model_path"
    trap - EXIT
    printf 'Model verified: %s\n' "$model_path"
fi

printf 'Camera dependencies ready in %s\n' "$venv_dir"
if (( ! download_model )); then
    printf 'Model download skipped. Use --download-model only when explicitly requested.\n'
fi
