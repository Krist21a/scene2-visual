#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/.."
export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1
export PYTHONPATH="$PWD:$PWD/vendor/scene1:${PYTHONPATH:-}"
exec "${SCENE2_PYTHON:-.venv/bin/python}" -m unittest discover -s tests -v
