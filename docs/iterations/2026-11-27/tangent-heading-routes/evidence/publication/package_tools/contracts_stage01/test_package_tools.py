"""Synthetic archive/restore contracts; no original campaign data are read."""
from contextlib import redirect_stdout
import io
import json
from pathlib import Path
import sys
import tarfile
import tempfile
import unittest

import numpy as np

sys.path.insert(0,str(Path(__file__).resolve().parent))
import build_record as build
import restore_record as restore


def binding(path):return dict(bytes=path.stat().st_size,sha256=restore.sha(path))


def write(path,data):
    path.parent.mkdir(parents=True,exist_ok=True)
    if isinstance(data,bytes):path.write_bytes(data)
    else:path.write_text(json.dumps(data,sort_keys=True))


def fixture(folder,bundle=False):
    root=Path(folder);evidence=root/'evidence';repo=root/'repo';repo.mkdir()
    files={'adapter/example.py':b'VALUE = 7\n','protocol/protocol.json':b'{"iteration":"I-76"}\n',
           'publication/frozen_inputs/control_schedule.npz':b'known frozen cache bytes'}
    for name,data in files.items():write(evidence/name,data)
    dependency=dict(sources={},inputs={},gain_cache=dict(path='generated/offroad/flat/control_schedule.npz',
        sha256=restore.sha(evidence/'publication/frozen_inputs/control_schedule.npz')),
        bridge_binary=dict(path='build/gazebo/libSphericalWheelBridge.so',sha256='none'),
        exact_physics_binary=dict(path='build/gazebo/libAswExactPhysics.so',sha256='none'))
    write(evidence/'publication/runtime_dependency_bindings.json',dependency)
    raw_bundle=None
    if bundle:
        raw_bundle=root/'raw.tar.gz';raw=root/'raw_input.npz';raw.write_bytes(b'lossless original npz bytes')
        origin=str(restore.RESULT_RELATIVE/'campaign/runs/m100_N/precision.npz')
        member='repository/'+origin
        with tarfile.open(raw_bundle,'w:gz') as archive:archive.add(raw,arcname=member,recursive=False)
        write(evidence/'publication/local_raw_bundle_inventory.json',dict(**binding(raw_bundle),files=[dict(
            origin=origin,archive_member=member,restore_kind='result_raw',**binding(raw))]))
    manifest=dict(result_relative=str(restore.RESULT_RELATIVE),
        frozen_gain_cache_relative='publication/frozen_inputs/control_schedule.npz',
        bundle_inventory='publication/local_raw_bundle_inventory.json',
        files={str(path.relative_to(evidence)):binding(path) for path in evidence.rglob('*') if path.is_file()})
    write(evidence/'compact_record_manifest.json',manifest)
    return evidence,repo,raw_bundle


class PackageContracts(unittest.TestCase):
    def call(self,*args,**kwargs):
        with redirect_stdout(io.StringIO()):return restore.restore(*args,**kwargs)

    def test_restore_and_repeat_preserve_bytes_and_unrelated_repo_file(self):
        with tempfile.TemporaryDirectory() as folder:
            evidence,repo,_=fixture(folder);write(repo/'src/unrelated.py',b'unchanged\n')
            self.call(evidence,repo);self.call(evidence,repo)
            target=repo/restore.RESULT_RELATIVE
            self.assertEqual((target/'adapter/example.py').read_bytes(),b'VALUE = 7\n')
            self.assertEqual((repo/'src/unrelated.py').read_bytes(),b'unchanged\n')
            self.assertFalse((repo/'generated/offroad/flat/control_schedule.npz').exists())

    def test_conflict_refuses_before_other_owned_files_are_written(self):
        with tempfile.TemporaryDirectory() as folder:
            evidence,repo,_=fixture(folder);target=repo/restore.RESULT_RELATIVE
            write(target/'adapter/example.py',b'keep differing local work\n')
            with self.assertRaises(ValueError):self.call(evidence,repo)
            self.assertEqual((target/'adapter/example.py').read_bytes(),b'keep differing local work\n')
            self.assertFalse((target/'protocol/protocol.json').exists())

    def test_evidence_corruption_rejected_without_writes(self):
        with tempfile.TemporaryDirectory() as folder:
            evidence,repo,_=fixture(folder);(evidence/'adapter/example.py').write_bytes(b'corrupted')
            with self.assertRaises(ValueError):self.call(evidence,repo)
            self.assertFalse((repo/restore.RESULT_RELATIVE).exists())

    def test_safe_paths_and_symlink_escape_rejected(self):
        for value in ('../src/controller.py','/tmp/other'):
            with self.subTest(value=value):
                with self.assertRaises(ValueError):restore.safe_relative(value)
        with tempfile.TemporaryDirectory() as folder:
            evidence,repo,_=fixture(folder);outside=Path(folder)/'outside';outside.mkdir()
            target=repo/restore.RESULT_RELATIVE;target.mkdir(parents=True)
            (target/'adapter').symlink_to(outside,target_is_directory=True)
            with self.assertRaises(ValueError):self.call(evidence,repo)
            self.assertEqual(list(outside.iterdir()),[])

    def test_lossless_raw_bundle_and_optional_cache_conflict(self):
        with tempfile.TemporaryDirectory() as folder:
            evidence,repo,bundle=fixture(folder,True)
            result=self.call(evidence,repo,bundle=bundle,install_cache=True)
            self.assertEqual(result['raw_files_restored'],1)
            self.assertFalse(result['full_raw_metric_files_present'])
            self.assertEqual((repo/restore.RESULT_RELATIVE/'campaign/runs/m100_N/precision.npz').read_bytes(),b'lossless original npz bytes')
            cache=repo/'generated/offroad/flat/control_schedule.npz';self.assertEqual(cache.read_bytes(),b'known frozen cache bytes')
            cache.write_bytes(b'preserve other gains')
            with self.assertRaises(ValueError):self.call(evidence,repo,install_cache=True)
            self.assertEqual(cache.read_bytes(),b'preserve other gains')

    def test_plot_only_decimation_keeps_real_terminal_and_original_indices(self):
        np.testing.assert_array_equal(build.decimation_indices(24),[0,10,20,23])
        np.testing.assert_array_equal(build.decimation_indices(21),[0,10,20])
        np.testing.assert_array_equal(build.decimation_indices(1),[0])
        with self.assertRaises(ValueError):build.decimation_indices(0)


if __name__=='__main__':unittest.main()
