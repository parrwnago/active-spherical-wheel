"""Prepare twelve fresh I52 jobs and provenance; never start Gazebo."""
import argparse
import json
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from research.workspace import WorkspaceIntegrityError, prepare_campaign


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True, help="New output directory beneath results/")
    args = parser.parse_args()
    try:
        report = prepare_campaign(ROOT, args.output)
    except (WorkspaceIntegrityError, FileExistsError, FileNotFoundError, KeyError, ValueError) as error:
        print(str(error), file=sys.stderr)
        return 2
    print(json.dumps(report, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
