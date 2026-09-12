#!/usr/bin/env bash
set -eu
cd "$(dirname "$0")"
python_bin=python3
if [[ -x .venv/bin/python ]]; then
    python_bin=.venv/bin/python
fi
exec "$python_bin" app.py "$@"
