#!/usr/bin/env python3
"""Copy the immutable I-72 result tree with an explicit, verified export plan.

Only the supplied source tree is read. Historical external references remain
metadata. Source-relative paths are retained without relocation.
"""
import argparse
import datetime
import hashlib
import json
import shutil
from pathlib import Path


SOURCE_PREFIX = "results/geometry_first_contact_20261127/"
REPLAY_DUPLICATES = {
    f"publication/replay_qa/stage01/{case}/predictions.npz"
    for case in ("m100_B", "p030_B", "p090_B")
}


def digest(path):
    h = hashlib.sha256()
    with path.open("rb") as f:
        for block in iter(lambda: f.read(1024 * 1024), b""):
            h.update(block)
    return h.hexdigest()


def write_json(path, value):
    path.write_text(json.dumps(value, indent=2, sort_keys=True) + "\n")


def inventory(source):
    entries = []
    for path in sorted(source.rglob("*")):
        if path.is_symlink():
            raise ValueError(f"Symlink is outside the allowed export model: {path}")
        if not path.is_file():
            continue
        relative = path.relative_to(source).as_posix()
        if "credential" in relative.lower():
            raise ValueError("Refusing to open a credential-named file")
        reason = None
        if "__pycache__" in path.relative_to(source).parts:
            reason = "Interpreter bytecode cache; source is preserved."
        elif relative in REPLAY_DUPLICATES:
            reason = (
                "Regenerated replay duplicate; canonical inference predictions and "
                "the original byte-exact replay receipt/bindings are retained."
            )
        item = {"path": relative, "bytes": path.stat().st_size,
                "sha256": digest(path), "action": "omit" if reason else "copy"}
        if reason:
            item["reason"] = reason
        entries.append(item)
    return entries


def inside(source, candidate):
    # Resolve only paths constructed from the allowed base, never external roots.
    path = candidate.resolve()
    return path if path.is_relative_to(source) else None


def resolve_binding(source, manifest, name):
    """Honor both manifest-relative and experiment-root-relative path origins."""
    name = str(name)
    if name.startswith(SOURCE_PREFIX):
        path = inside(source, source / name[len(SOURCE_PREFIX):])
        return path, "experiment-root prefix"
    if Path(name).is_absolute() or name.startswith(("results/", "src/", "tests/", "docs/")):
        return None, "external original path; not opened"
    candidates = [(manifest.parent / name, "manifest folder"),
                  (source / name, "experiment root")]
    for candidate, origin in candidates:
        path = inside(source, candidate)
        if path is not None and path.is_file():
            return path, origin
    return None, "unresolved local binding"


def file_records(data):
    records = []
    values = data.get("files")
    if isinstance(values, list):
        records += [x for x in values if isinstance(x, dict)]
    elif isinstance(values, dict):
        records += [{"path": key, **value} for key, value in values.items()
                    if isinstance(value, dict)]
    elif values is None:
        records += [{"path": key, **value} for key, value in data.items()
                    if isinstance(value, dict) and "sha256" in value]
    if "copy" in data and "sha256" in data:
        records.append({"path": data["copy"], "sha256": data["sha256"]})
    records += [x for x in data.get("omitted_recomputed_predictions", [])
                if isinstance(x, dict)]
    return records


def audit_bindings(source, entries):
    checks, external, errors = [], [], []

    def check(manifest, name, expected, expected_bytes=None, category="artifact manifest"):
        path, origin = resolve_binding(source, manifest, name)
        entry = {"manifest": manifest.relative_to(source).as_posix(),
                 "recorded_path": name, "path_origin": origin,
                 "expected_sha256": expected, "category": category}
        if path is None:
            if origin == "unresolved local binding":
                entry["passed"] = False
                errors.append(entry)
            else:
                entry["status"] = "external reference retained; not opened"
                external.append(entry)
            return
        entry["resolved_path"] = path.relative_to(source).as_posix()
        entry["actual_sha256"] = digest(path)
        entry["passed"] = entry["actual_sha256"] == expected
        if expected_bytes is not None:
            entry["expected_bytes"] = expected_bytes
            entry["actual_bytes"] = path.stat().st_size
            entry["passed"] &= entry["actual_bytes"] == expected_bytes
        checks.append(entry)
        if not entry["passed"]:
            errors.append(entry)

    for item in entries:
        relative = item["path"]
        path = source / relative
        if path.name.endswith("manifest.json") or path.name == "all_three_sealed.json":
            data = json.loads(path.read_text())
            for record in file_records(data):
                name = record.get("path", record.get("copy"))
                if name is not None and "sha256" in record:
                    check(path, name, record["sha256"], record.get("bytes"))
        if path.name == "render_receipt.json":
            data = json.loads(path.read_text())
            for name, expected in data.get("outputs", {}).items():
                if isinstance(expected, str):
                    check(path, name, expected, category="render output binding")

    seal = source / "protocol/inference_source_input_seal.json"
    data = json.loads(seal.read_text())
    for group in ("sources", "input_files"):
        for name, expected in data[group].items():
            check(seal, name, expected, category=f"final inference seal: {group}")

    # Check the compact, explicitly copied 57-field evaluation packet bindings.
    # This is byte verification only: no labels, metrics, or scores are computed.
    import numpy as np
    metadata = source / "publication/evaluation_inputs/metadata.json"
    local_packet_arrays = []
    for record in json.loads(metadata.read_text())["files"]:
        path, origin = resolve_binding(source, metadata, record["path"])
        if path is None:
            raise ValueError("Evaluation input copy must resolve inside the source tree")
        check(metadata, record["path"], record["sha256"], record["bytes"],
              "copied evaluation input")
        # Retain the original reference as metadata without opening that path.
        external.append({"manifest": metadata.relative_to(source).as_posix(),
                         "recorded_path": record["original_path"],
                         "expected_sha256": record["original_sha256"],
                         "status": "external original retained; not opened"})
        with np.load(path, allow_pickle=False) as packet:
            if set(packet.files) != set(record["arrays"]):
                raise ValueError(f"Unexpected fields in {path.relative_to(source)}")
            for key, binding in record["arrays"].items():
                value = packet[key]
                actual = hashlib.sha256(value.tobytes(order="C")).hexdigest()
                passed = (actual == binding["sha256"] and
                          value.dtype.str == binding["dtype"] and
                          list(value.shape) == binding["shape"])
                result = {"path": path.relative_to(source).as_posix(), "field": key,
                          "dtype": value.dtype.str, "shape": list(value.shape),
                          "sha256": actual, "passed": passed}
                local_packet_arrays.append(result)
                if not passed:
                    errors.append(result)
    return {"status": "PASS" if not errors else "FAIL", "file_checks": checks,
            "verified_file_bindings": len(checks),
            "evaluation_array_checks": local_packet_arrays,
            "verified_evaluation_array_bindings": len(local_packet_arrays),
            "external_references_not_opened": external, "errors": errors,
            "scope": "Original artifact manifests, final all-three seals, plot output "
                     "bindings, final inference seal in-scope references, and copied "
                     "evaluation-input file/array bindings. Historical failed source "
                     "stages remain unchanged with their own source snapshots; earlier "
                     "seals are preserved rather than interpreted as current sources."}


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--source", required=True, type=Path)
    args = parser.parse_args()
    source = args.source.resolve()
    target = Path(__file__).resolve().parent
    if source.name != "geometry_first_contact_20261127" or source.parent.name != "results":
        raise ValueError("Unexpected source experiment")
    for name in ("export_plan.json", "export_manifest.json", "transfer_receipt.json"):
        if (target / name).exists():
            raise FileExistsError(f"Preserve the existing export stage before rerunning: {name}")
    for relative in ("review/combined_contracts_stage01/receipt.json",
                     "publication/plots_stage02/render_receipt.json"):
        if not (source / relative).is_file():
            raise FileNotFoundError(f"Final source readiness guard missing: {relative}")
    entries = inventory(source)
    omitted = [x for x in entries if x["action"] == "omit"]
    included = [x for x in entries if x["action"] == "copy"]
    actual_replays = {x["path"] for x in omitted if x["path"] in REPLAY_DUPLICATES}
    if actual_replays != REPLAY_DUPLICATES:
        raise ValueError("Expected exactly the three specified regenerated predictions")
    plan = {"registered_utc": datetime.datetime.now(datetime.timezone.utc).isoformat(),
            "source": SOURCE_PREFIX.rstrip("/"),
            "destination": "docs/iterations/2026-11-27/geometry-first-contact/evidence",
            "path_policy": "Identity: every included path remains relative to the I72 base.",
            "registered_before_copy": True, "files": entries,
            "copy_file_count": len(included), "copy_bytes": sum(x["bytes"] for x in included),
            "omitted_file_count": len(omitted), "omitted_bytes": sum(x["bytes"] for x in omitted)}
    write_json(target / "export_plan.json", plan)
    audit = audit_bindings(source, entries)
    write_json(target / "artifact_binding_audit.json", audit)
    if audit["status"] != "PASS":
        raise ValueError("Original artifact binding preflight failed; preserve this stage")
    for item in included:
        original, copied = source / item["path"], target / item["path"]
        if digest(original) != item["sha256"] or original.stat().st_size != item["bytes"]:
            raise ValueError(f"Source changed after plan: {item['path']}")
        copied.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(original, copied)
        if digest(copied) != item["sha256"] or copied.stat().st_size != item["bytes"]:
            raise ValueError(f"Copied bytes failed verification: {item['path']}")
    if inventory(source) != entries:
        raise ValueError("Source inventory changed during export")
    support = []
    for name in ("transfer_evidence.py", "export_plan.json", "artifact_binding_audit.json",
                 "TRANSFER_README.md"):
        path = target / name
        support.append({"path": name, "bytes": path.stat().st_size, "sha256": digest(path)})
    manifest = {"status": "PASS", "source": SOURCE_PREFIX.rstrip("/"),
                "copied_files": [{k: x[k] for k in ("path", "bytes", "sha256")}
                                 for x in included],
                "omitted_files": omitted, "generated_support_files": support,
                "copy_file_count": len(included), "copy_bytes": plan["copy_bytes"],
                "scope": "All source-relative paths retained; no scientific rerun, "
                         "reformatting, or changes. Manifest and transfer receipt are "
                         "not self-hashed; receipt binds this manifest."}
    write_json(target / "export_manifest.json", manifest)
    receipt = {"status": "PASS", "completed_utc": datetime.datetime.now(datetime.timezone.utc).isoformat(),
               "export_plan_sha256": digest(target / "export_plan.json"),
               "export_manifest_sha256": digest(target / "export_manifest.json"),
               "artifact_binding_audit_sha256": digest(target / "artifact_binding_audit.json"),
               "copied_files": len(included), "copied_bytes": plan["copy_bytes"],
               "omitted_files": len(omitted), "omitted_bytes": plan["omitted_bytes"],
               "original_file_bindings_verified": audit["verified_file_bindings"],
               "evaluation_array_bindings_verified": audit["verified_evaluation_array_bindings"],
               "all_copied_files_byte_exact": True, "source_inventory_unchanged": True,
               "external_reference_files_opened": False,
               "setup_note": "Initial shell inventory used unavailable python; no source "
                             "or export artifacts changed. Subsequent inventory and export "
                             "used python3. No experiment was rerun.",
               "no_scientific_changes_or_computation": True}
    write_json(target / "transfer_receipt.json", receipt)
    print(json.dumps(receipt, indent=2, sort_keys=True))


if __name__ == "__main__":
    main()
