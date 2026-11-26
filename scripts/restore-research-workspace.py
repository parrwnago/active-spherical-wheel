"""Restore byte-exact research prerequisites; --check never writes files."""
import argparse
import json
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from research.workspace import WorkspaceIntegrityError, restore_workspace


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--check", action="store_true", help="Only verify; return 1 when valid prerequisite targets are missing")
    args = parser.parse_args()
    try:
        report = restore_workspace(ROOT, check=args.check)
    except (WorkspaceIntegrityError, FileNotFoundError, KeyError, ValueError) as error:
        print(str(error), file=sys.stderr)
        return 2
    print(json.dumps(report, indent=2))
    return 0 if report["ready"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
