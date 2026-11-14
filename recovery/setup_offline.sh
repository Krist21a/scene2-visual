#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/.."
python_bin="${PYTHON_BIN:-python3}"
"$python_bin" -c 'import sys; assert (3,10)<=sys.version_info[:2]<=(3,12), "Use Python 3.10–3.12; archived wheels require CPython 3.12 on Linux x86_64"'
"$python_bin" -m venv .venv
if compgen -G 'downloads/wheels/*.whl' >/dev/null; then
  .venv/bin/python -m pip install --no-index --find-links downloads/wheels -r requirements-offline.txt
else
  .venv/bin/python -m pip install -r requirements-offline.txt
fi
.venv/bin/python -m pip check
python3 recovery/verify.py
echo 'Ready: bash recovery/run_tests.sh'
