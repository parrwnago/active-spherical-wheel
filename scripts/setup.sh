#!/usr/bin/env bash
set -euo pipefail
task_repo_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$task_repo_dir"
if ! python3 -m venv .venv; then
    # Minimal Linux images can provide pip but omit ensurepip. This fallback
    # bootstraps only the local environment and needs no system modifications.
    python3 -m venv --without-pip .venv
    task_site_dir="$(.venv/bin/python -c 'import sysconfig; print(sysconfig.get_path("purelib"))')"
    python3 -m pip install --target "$task_site_dir" pip==26.2.1
fi
.venv/bin/python -m pip install -r requirements-lock.txt
.venv/bin/python -m pip install -e . --no-deps --no-build-isolation
printf '\nReady: .venv/bin/python -m spherical_wheel balance\n'
