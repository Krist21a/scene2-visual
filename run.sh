#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")"
set +u
source /opt/ros/noetic/setup.bash
set -u
export ROS_MASTER_URI=http://127.0.0.1:11311
export ROS_IP=127.0.0.1
export PYTHONPATH="/root/LeTools-Learning:${PYTHONPATH:-}"
export OPENBLAS_NUM_THREADS=1
export OMP_NUM_THREADS=1
exec /root/miniforge3/envs/kls_dev/bin/python "$@"
