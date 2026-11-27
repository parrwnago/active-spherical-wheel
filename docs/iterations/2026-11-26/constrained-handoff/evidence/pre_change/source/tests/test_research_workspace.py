"""Pure handoff contracts: synthetic tiny files, no model or physics imports."""
from copy import deepcopy
import json
from pathlib import Path

import pytest

from research.workspace import WorkspaceIntegrityError, prepare_campaign, restore_workspace, sha256
from research import workspace as workspace_module


def put(path, content):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_bytes(content)
    return path


@pytest.fixture
def workspace(tmp_path):
    root = tmp_path / "clone"
    payload = root / "research/frozen_workspace"
    live = put(root / "src/native.py", b"# synthetic immutable live source\n")
    family = "results/contact_scheduling_joint_tracking_20261126"
    cases = [dict(case_id=p + "_" + v, cap_m_s=.3) for p in ("m100", "p030", "p090") for v in ("B", "R", "N", "NR")]
    protocol = dict(iteration="I-52", cases=cases, execution=dict(max_parallel_native_cases=3))
    contents = {
        family + "/protocol.json": json.dumps(protocol, indent=2).encode() + b"\n",
        family + "/campaign/jobs.json": json.dumps(dict(cases=cases), indent=2).encode() + b"\n",
        family + "/run_factorial_case.py": b"raise AssertionError('Must never be executed by preparation')\n",
        "generated/offroad/flat/control_schedule.npz": b"synthetic gain fixture; not a numerical model",
    }
    entries = []
    for target, content in contents.items():
        store = "files/" + target
        source = put(payload / store, content)
        entries.append(dict(store_path=store, target=target, origin=target, bytes=len(content),
                            sha256=sha256(source), restore=True, role="fixture"))
    snapshot = put(payload / "files/src/native.py", live.read_bytes())
    entries.append(dict(store_path="files/src/native.py", target="src/native.py", origin="src/native.py",
                        bytes=snapshot.stat().st_size, sha256=sha256(snapshot), restore=False, role="live_source_snapshot"))
    manifest = dict(schema_version=1, iteration="I-52", protocol_target=family + "/protocol.json",
                    jobs_target=family + "/campaign/jobs.json", runner_target=family + "/run_factorial_case.py",
                    entries=entries, required_live_source_sha256={"src/native.py": sha256(live)})
    (payload / "manifest.json").write_text(json.dumps(manifest))
    return root, payload, manifest


def update_manifest(payload, manifest):
    (payload / "manifest.json").write_text(json.dumps(manifest))


def test_readonly_check_does_not_create_missing_targets(workspace):
    root, payload, _ = workspace
    before = sorted(str(p.relative_to(root)) for p in root.rglob("*"))
    report = restore_workspace(root, payload, check=True)
    assert not report["ready"] and report["restored_files"] == 0
    assert len(report["missing_targets"]) == 4
    assert not (root / "results").exists() and not (root / "generated").exists()
    assert sorted(str(p.relative_to(root)) for p in root.rglob("*")) == before


def test_restore_is_byte_exact_and_repeat_is_noop(workspace):
    root, payload, manifest = workspace
    first = restore_workspace(root, payload)
    assert first["ready"] and first["restored_files"] == 4
    before = {entry["target"]: (root / entry["target"]).stat().st_mtime_ns for entry in manifest["entries"]}
    second = restore_workspace(root, payload)
    assert second["ready"] and second["restored_files"] == 0
    for entry in manifest["entries"]:
        target = root / entry["target"]
        assert sha256(target) == entry["sha256"]
        assert target.stat().st_mtime_ns == before[entry["target"]]


def test_differing_existing_target_aborts_before_any_copy(workspace):
    root, payload, manifest = workspace
    last_target = manifest["entries"][-2]["target"]
    differing = put(root / last_target, b"keep teammate's differing gain data")
    with pytest.raises(WorkspaceIntegrityError, match="Refusing to replace"):
        restore_workspace(root, payload)
    assert differing.read_bytes() == b"keep teammate's differing gain data"
    assert not (root / "results").exists()


def test_corrupt_payload_aborts_before_creating_target_dirs(workspace):
    root, payload, manifest = workspace
    (payload / manifest["entries"][0]["store_path"]).write_bytes(b"corrupt")
    with pytest.raises(WorkspaceIntegrityError, match="payload"):
        restore_workspace(root, payload)
    assert not (root / "results").exists()


def test_changed_live_source_is_preserved_and_prevents_restore(workspace):
    root, payload, _ = workspace
    native = root / "src/native.py"
    native.write_bytes(b"# local experiment changes\n")
    with pytest.raises(WorkspaceIntegrityError, match="live source"):
        restore_workspace(root, payload)
    assert native.read_bytes() == b"# local experiment changes\n"
    assert not (root / "results").exists()


@pytest.mark.parametrize("field,value", [
    ("target", "results/../../outside"), ("target", "/tmp/outside"),
    ("target", "results//noncanonical"), ("target", "src/overwrite.py"),
    ("store_path", "../outside"), ("store_path", "files/../../outside"),
])
def test_manifest_rejects_path_escape_or_nonignored_target(workspace, field, value):
    root, payload, manifest = workspace
    altered = deepcopy(manifest)
    altered["entries"][0][field] = value
    update_manifest(payload, altered)
    with pytest.raises(WorkspaceIntegrityError):
        restore_workspace(root, payload)
    assert not (root / "results").exists()


def test_symlinked_destination_directory_cannot_escape_clone(workspace, tmp_path):
    root, payload, _ = workspace
    external = tmp_path / "external"
    external.mkdir()
    (root / "results").symlink_to(external, target_is_directory=True)
    with pytest.raises(WorkspaceIntegrityError, match="escapes|[Ss]ymlink"):
        restore_workspace(root, payload)
    assert list(external.iterdir()) == []


@pytest.mark.parametrize("redirect", ["results", "results/contact_scheduling_joint_tracking_20261126"])
def test_inside_repo_symlink_cannot_redirect_restore_into_live_source(workspace, redirect):
    root, payload, _ = workspace
    link = root / redirect
    link.parent.mkdir(parents=True, exist_ok=True)
    link.symlink_to(root / "src", target_is_directory=True)
    before = {path.name: path.read_bytes() for path in (root / "src").iterdir()}
    with pytest.raises(WorkspaceIntegrityError, match="[Ss]ymlink"):
        restore_workspace(root, payload)
    assert {path.name: path.read_bytes() for path in (root / "src").iterdir()} == before


def test_even_identical_existing_symlink_restore_targets_are_refused(workspace):
    root, payload, manifest = workspace
    entry = manifest["entries"][0]
    target = root / entry["target"]
    target.parent.mkdir(parents=True, exist_ok=True)
    target.symlink_to(payload / entry["store_path"])
    with pytest.raises(WorkspaceIntegrityError, match="[Ss]ymlink"):
        restore_workspace(root, payload)


def test_duplicate_manifest_target_is_rejected(workspace):
    root, payload, manifest = workspace
    manifest["entries"].append(deepcopy(manifest["entries"][0]))
    update_manifest(payload, manifest)
    with pytest.raises(WorkspaceIntegrityError, match="Duplicate"):
        restore_workspace(root, payload)
    assert not (root / "results").exists()


def test_prepare_requires_completed_restore_and_has_no_implicit_copy(workspace):
    root, payload, _ = workspace
    with pytest.raises(WorkspaceIntegrityError, match="Restore"):
        prepare_campaign(root, Path("results/fresh"), payload)
    assert not (root / "results").exists()


def test_prepare_creates_exact_protocol_and_fresh_twelve_case_resolver(workspace):
    root, payload, manifest = workspace
    restore_workspace(root, payload)
    output = root / "results/reproductions/example"
    result = prepare_campaign(root, output, payload)
    assert result["prepared"] and result["case_count"] == 12 and result["native_executions"] == 0
    assert (output / "protocol.json").read_bytes() == (root / manifest["protocol_target"]).read_bytes()
    assert (output / "campaign/jobs.json").read_bytes() == (root / manifest["jobs_target"]).read_bytes()
    resolver = json.loads((output / "campaign/resolved_case_paths.json").read_text())
    assert resolver["dataset_kind"] == "fresh_reproduction" and resolver["technical_attempts"] == []
    assert len(resolver["cases"]) == 12
    assert all(path.startswith("results/reproductions/example/campaign/runs/") for path in resolver["cases"].values())
    assert not (output / "campaign/runs").exists()
    registration = json.loads((output / "reproduction_registration.json").read_text())
    assert registration["native_executions"] == 0 and registration["technical_attempts"] == []
    assert (output / "RUN_COMMANDS.md").read_text().count(" --run") == 12


def test_prepare_refuses_existing_output_and_preserves_it(workspace):
    root, payload, _ = workspace
    restore_workspace(root, payload)
    output = root / "results/existing"
    sentinel = put(output / "keep.txt", b"do not replace")
    with pytest.raises(FileExistsError):
        prepare_campaign(root, output, payload)
    assert sentinel.read_bytes() == b"do not replace"
    assert not (output / "protocol.json").exists()


def test_prepare_rejects_origin_change_after_initial_validation(workspace, monkeypatch):
    root, payload, manifest = workspace
    restore_workspace(root, payload)
    original_restore = workspace_module.restore_workspace

    def changed_after_validation(*args, **kwargs):
        report = original_restore(*args, **kwargs)
        protocol = root / manifest["protocol_target"]
        protocol.write_bytes(protocol.read_bytes() + b"\n")
        return report

    monkeypatch.setattr(workspace_module, "restore_workspace", changed_after_validation)
    with pytest.raises(WorkspaceIntegrityError, match="protocol/jobs changed"):
        prepare_campaign(root, Path("results/concurrent_change"), payload)
    assert not (root / "results/concurrent_change").exists()


def test_prepare_rejects_output_outside_ignored_results(workspace, tmp_path):
    root, payload, _ = workspace
    restore_workspace(root, payload)
    with pytest.raises(WorkspaceIntegrityError, match="beneath"):
        prepare_campaign(root, tmp_path / "outside", payload)
    with pytest.raises(WorkspaceIntegrityError, match="beneath"):
        prepare_campaign(root, Path("research/new_campaign"), payload)
    assert not (root / "research/new_campaign").exists()
