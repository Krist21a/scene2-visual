#!/usr/bin/env bash
# Run inside a compatible ROS Noetic container after downloading runtime assets.
set -euo pipefail
cd "$(dirname "$0")/.."
test -f /opt/ros/noetic/setup.bash
test -x /root/miniforge3/bin/conda || {
  echo 'Install Miniforge at /root/miniforge3 first; see docs/RECOVERY.md.' >&2
  exit 1
}
test ! -e /root/miniforge3/envs/kls_dev || {
  echo 'kls_dev already exists. This restore script only creates a fresh environment.' >&2
  exit 1
}
test ! -e /root/LeTools-Learning || {
  echo '/root/LeTools-Learning already exists; restore to a fresh container.' >&2
  exit 1
}
python3 recovery/verify.py --runtime
/root/miniforge3/bin/conda create -y -p /root/miniforge3/envs/kls_dev --file recovery/conda-production-explicit.txt
/root/miniforge3/envs/kls_dev/bin/python -m pip install --no-deps -r recovery/pip-production-nodeps.txt
cp -a runtime/LeTools-Learning /root/LeTools-Learning
/root/miniforge3/envs/kls_dev/bin/python -m pip install --no-deps -e /root/LeTools-Learning
# Preserve the captured SDK implementation, including generated ROS messages.
/root/miniforge3/envs/kls_dev/bin/python - <<'PY'
import shutil,sysconfig
from pathlib import Path
site=Path(sysconfig.get_paths()['purelib'])
for name in ('kuavo_humanoid_sdk','kuavo_humanoid_sdk-1.3.3.dist-info'):
    destination=site/name
    if destination.exists():shutil.rmtree(destination)
    shutil.copytree(Path('runtime')/name,destination)
PY
mkdir -p evidence outputs archives reports
echo 'Runtime installed. Review ROS/GPU/X11 and simulator setup before starting any task.'
