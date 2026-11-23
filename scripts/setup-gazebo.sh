#!/usr/bin/env bash
# Prepare the Gazebo reconstruction: Python packages, plugins and the Mars yard.
# Run inside the asw-gz conda environment (environment/gazebo.yml).
set -euo pipefail
cd "$(dirname "$0")/.."
: "${CONDA_PREFIX:?Activate the asw-gz environment first: conda activate asw-gz}"
python -m pip install -q -r requirements-lock.txt
python -m pip install -q --no-deps -e .
cmake -S gazebo -B build/gazebo -DCMAKE_PREFIX_PATH="$CONDA_PREFIX" -DCMAKE_BUILD_TYPE=Release
cmake --build build/gazebo -j"$(nproc)"
bash scripts/fetch-marsyard.sh
echo "Ready. Check: PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python -m pytest -q tests/test_gazebo.py"
