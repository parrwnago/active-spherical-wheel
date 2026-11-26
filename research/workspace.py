"""Restore hash-bound I52 prerequisites and prepare fresh, unexecuted cases.

This module uses only the standard library.  It never imports the controller,
starts a simulator, downloads assets, changes an existing file or runs a case.
The historical files in frozen_workspace retain their original bytes.
"""
from __future__ import annotations

from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path, PurePosixPath
import re
import shlex


class WorkspaceIntegrityError(ValueError):
    """A path, payload, live source or existing target violates the manifest."""


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def _relative(value: str) -> PurePosixPath:
    if not isinstance(value, str) or not value or "\\" in value or "\0" in value:
        raise WorkspaceIntegrityError("Expected a canonical repository-relative POSIX path")
    path = PurePosixPath(value)
    if path.is_absolute() or any(part in (".", "..") for part in path.parts):
        raise WorkspaceIntegrityError(f"Unsafe relative path: {value}")
    if path.as_posix() != value or not path.parts or ":" in path.parts[0]:
        raise WorkspaceIntegrityError(f"Noncanonical relative path: {value}")
    return path


def _inside(root: Path, relative: str) -> Path:
    path = root / _relative(relative)
    if not path.resolve().is_relative_to(root.resolve()):
        raise WorkspaceIntegrityError(f"Path escapes its root: {relative}")
    return path


def _restorable(relative: str) -> bool:
    parts = _relative(relative).parts
    return len(parts) >= 2 and (parts[0] == "results" or parts[:2] == ("generated", "offroad"))


def _restore_destination(root: Path, relative: str) -> Path:
    if not _restorable(relative):
        raise WorkspaceIntegrityError(f"Restore target is outside ignored research paths: {relative}")
    cursor = root
    for part in _relative(relative).parts:
        cursor = cursor / part
        # Containment within the repo alone would allow results/foo -> src/.
        # Refuse every destination symlink, including identical file aliases.
        if cursor.is_symlink():
            raise WorkspaceIntegrityError(f"Symlinked restore destination is refused: {relative}")
    return _inside(root, relative)


def load_manifest(payload: Path) -> dict:
    payload = Path(payload).resolve()
    manifest = json.loads((payload / "manifest.json").read_text())
    if manifest.get("schema_version") != 1 or manifest.get("iteration") != "I-52":
        raise WorkspaceIntegrityError("Unsupported frozen workspace manifest")
    entries = manifest.get("entries")
    if not isinstance(entries, list) or not entries:
        raise WorkspaceIntegrityError("Manifest has no payload entries")
    targets, stores = set(), set()
    for entry in entries:
        target, store = entry["target"], entry["store_path"]
        _relative(target)
        _inside(payload, store)
        if target in targets or store in stores:
            raise WorkspaceIntegrityError("Duplicate payload target or store path")
        targets.add(target)
        stores.add(store)
        if not re.fullmatch(r"[0-9a-f]{64}", entry.get("sha256", "")):
            raise WorkspaceIntegrityError(f"Invalid SHA-256: {target}")
        if type(entry.get("bytes")) is not int or entry["bytes"] < 0:
            raise WorkspaceIntegrityError(f"Invalid byte count: {target}")
        if type(entry.get("restore")) is not bool:
            raise WorkspaceIntegrityError(f"Missing restore policy: {target}")
        if entry["restore"] and not _restorable(target):
            raise WorkspaceIntegrityError(f"Restore target is outside ignored research paths: {target}")
    live = manifest.get("required_live_source_sha256")
    if not isinstance(live, dict) or not live:
        raise WorkspaceIntegrityError("Manifest must bind required live sources")
    for relative, expected in live.items():
        _relative(relative)
        if not re.fullmatch(r"[0-9a-f]{64}", expected):
            raise WorkspaceIntegrityError(f"Invalid live-source SHA-256: {relative}")
    for entry in entries:
        if not entry["restore"] and live.get(entry["target"]) != entry["sha256"]:
            raise WorkspaceIntegrityError("Reference-only payload must bind an identical live source")
    for key in ("protocol_target", "jobs_target", "runner_target"):
        if manifest.get(key) not in targets:
            raise WorkspaceIntegrityError(f"Manifest {key} is not a payload target")
    return manifest


def restore_workspace(root: Path, payload: Path | None = None, *, check: bool = False) -> dict:
    """Validate everything first; then copy missing targets using exclusive writes.

    ``check=True`` performs no writes and reports missing valid targets. Any
    differing existing target, changed payload or changed required live source
    raises before any target directory or file is created.
    """
    root = Path(root).resolve()
    payload = Path(payload).resolve() if payload is not None else root / "research/frozen_workspace"
    manifest = load_manifest(payload)
    records, missing = [], []
    for relative, expected in manifest["required_live_source_sha256"].items():
        path = _inside(root, relative)
        if not path.is_file() or sha256(path) != expected:
            raise WorkspaceIntegrityError(f"Required live source is missing or differs: {relative}")
    for entry in manifest["entries"]:
        source = _inside(payload, entry["store_path"])
        if not source.is_file() or source.stat().st_size != entry["bytes"] or sha256(source) != entry["sha256"]:
            raise WorkspaceIntegrityError(f"Frozen payload is missing or differs: {entry['store_path']}")
        target = (_restore_destination(root, entry["target"]) if entry["restore"]
                  else _inside(root, entry["target"]))
        if target.exists() or target.is_symlink():
            if not target.is_file() or sha256(target) != entry["sha256"]:
                raise WorkspaceIntegrityError(f"Refusing to replace differing existing target: {entry['target']}")
            status = "identical"
        elif entry["restore"]:
            status = "missing"
            missing.append(entry)
        else:
            raise WorkspaceIntegrityError(f"Required reference-only live source is missing: {entry['target']}")
        records.append(dict(target=entry["target"], status=status, restore=entry["restore"]))
    if not check:
        for entry in missing:
            target = _restore_destination(root, entry["target"])
            target.parent.mkdir(parents=True, exist_ok=True)
            # Exclusive creation refuses a concurrent writer as well as normal overwrites.
            with _inside(payload, entry["store_path"]).open("rb") as source, target.open("xb") as output:
                for block in iter(lambda: source.read(1024 * 1024), b""):
                    output.write(block)
            if target.stat().st_size != entry["bytes"] or sha256(target) != entry["sha256"]:
                raise WorkspaceIntegrityError(f"Copied target failed verification: {entry['target']}")
        for record in records:
            if record["status"] == "missing":
                record["status"] = "restored"
    return dict(iteration="I-52", check_only=check, ready=not missing or not check,
                manifest_sha256=sha256(payload / "manifest.json"),
                required_live_sources_verified=len(manifest["required_live_source_sha256"]),
                payload_files_verified=len(records), missing_targets=[entry["target"] for entry in missing],
                restored_files=0 if check else len(missing), records=records,
                no_controller_or_physics_execution=True)


def prepare_campaign(root: Path, output: Path, payload: Path | None = None) -> dict:
    """Create an unexecuted fresh campaign from the exact historical protocol/jobs.

    No original raw results or historical retry interpretation are copied. The
    fresh resolver has twelve planned case paths and no technical attempts.
    """
    root = Path(root).resolve()
    payload = Path(payload).resolve() if payload is not None else root / "research/frozen_workspace"
    manifest = load_manifest(payload)
    verification = restore_workspace(root, payload, check=True)
    if not verification["ready"]:
        raise WorkspaceIntegrityError("Restore the missing frozen prerequisites before preparing a campaign")
    output = Path(output)
    output = output if output.is_absolute() else root / output
    if not output.resolve().is_relative_to(root / "results") or output.resolve() == root / "results":
        raise WorkspaceIntegrityError("Fresh output must be a directory beneath this repository's results/")
    output = output.resolve()
    if output.exists() or output.is_symlink():
        raise FileExistsError(f"Refusing existing campaign output: {output}")
    protocol_source = _inside(root, manifest["protocol_target"])
    jobs_source = _inside(root, manifest["jobs_target"])
    expected = {entry["target"]: entry["sha256"] for entry in manifest["entries"]}
    protocol_bytes, jobs_bytes = protocol_source.read_bytes(), jobs_source.read_bytes()
    protocol_sha = hashlib.sha256(protocol_bytes).hexdigest()
    jobs_sha = hashlib.sha256(jobs_bytes).hexdigest()
    if protocol_sha != expected[manifest["protocol_target"]] or jobs_sha != expected[manifest["jobs_target"]]:
        raise WorkspaceIntegrityError("Frozen protocol/jobs changed after initial validation")
    protocol = json.loads(protocol_bytes)
    jobs = json.loads(jobs_bytes)
    if protocol.get("iteration") != "I-52" or jobs.get("cases") != protocol.get("cases"):
        raise WorkspaceIntegrityError("Frozen protocol and jobs disagree")
    cases = protocol.get("cases", [])
    identifiers = [job.get("case_id") for job in cases]
    expected = {phase + "_" + variant for phase in ("m100", "p030", "p090") for variant in ("B", "R", "N", "NR")}
    if len(identifiers) != 12 or set(identifiers) != expected:
        raise WorkspaceIntegrityError("Expected the twelve registered I52 policy cells")
    relative_output = output.relative_to(root).as_posix()
    resolution = dict(iteration="I-52", dataset_kind="fresh_reproduction", cases={
        case: f"{relative_output}/campaign/runs/{case}" for case in identifiers},
        technical_attempts=[], interpretation="Fresh planned dataset; no native cases have been executed. The original study's technical crash/retry is not part of this dataset.")
    adapter_sources = {}
    for relative in ("research/workspace.py", "scripts/restore-research-workspace.py", "scripts/prepare-research-campaign.py"):
        path = root / relative
        if path.is_file():
            adapter_sources[relative] = sha256(path)
    registration = dict(iteration="I-52", dataset_kind="fresh_reproduction",
        prepared_utc=datetime.now(timezone.utc).isoformat(), output=relative_output,
        original_protocol=dict(path=manifest["protocol_target"], sha256=protocol_sha),
        original_jobs=dict(path=manifest["jobs_target"], sha256=jobs_sha),
        frozen_workspace_manifest_sha256=verification["manifest_sha256"], adapter_source_sha256=adapter_sources,
        case_count=12, native_executions=0, technical_attempts=[],
        max_parallel_native_cases=protocol.get("execution", {}).get("max_parallel_native_cases", 3),
        analysis_scope="Native execution recipe only. Frozen scientific reducers preserve original dataset/provenance/retry assumptions and require a separately declared fresh-data adapter.")
    commands = ["python " + shlex.quote(manifest["runner_target"]) + " --output " + shlex.quote(relative_output)
                + " --cap 0.30 --case-id " + case + " --run" for case in identifiers]
    output.parent.mkdir(parents=True, exist_ok=True)
    output.mkdir(exist_ok=False)
    (output / "campaign").mkdir()
    # Both source files were verified above; exclusive writes preserve new-output semantics.
    for content, target in ((protocol_bytes, output / "protocol.json"), (jobs_bytes, output / "campaign/jobs.json")):
        with target.open("xb") as stream:
            stream.write(content)
    for target, value in ((output / "campaign/resolved_case_paths.json", resolution),
                          (output / "reproduction_registration.json", registration)):
        with target.open("x") as stream:
            json.dump(value, stream, indent=2)
            stream.write("\n")
    with (output / "RUN_COMMANDS.md").open("x") as stream:
        stream.write("# Fresh I52 campaign — prepared, not executed\n\nActivate the documented Gazebo environment, build/fetch its prerequisites, and run from the repository root.\n\nSet `OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 MUJOCO_GL=egl`; run at most three fresh processes concurrently.\n\nThe twelve independent native commands below each write a new case folder and refuse overwrites.\n\n```bash\n" + "\n".join(commands) + "\n```\n\nNo cases are started by preparation. Original technical-crash metadata and original outcome claims do not describe this fresh dataset. This recipe does not launch the frozen original-data scientific reducer.\n")
    if sha256(output / "protocol.json") != protocol_sha or sha256(output / "campaign/jobs.json") != jobs_sha:
        raise WorkspaceIntegrityError("Prepared protocol/jobs failed byte verification")
    return dict(prepared=True, output=relative_output, case_count=12, native_executions=0,
                protocol_sha256=protocol_sha, jobs_sha256=jobs_sha,
                resolver="campaign/resolved_case_paths.json", registration="reproduction_registration.json")
