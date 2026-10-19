#!/usr/bin/env bash
set -euo pipefail
task_repo_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$task_repo_dir"
if [[ ! -x .venv/bin/python ]]; then
    bash scripts/setup.sh
fi
exec .venv/bin/python -m spherical_wheel lab --open-browser "$@"
