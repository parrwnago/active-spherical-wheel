"""Read-only I58 registration contracts; no native controller/model/physics."""
from pathlib import Path
from importlib.util import spec_from_file_location,module_from_spec
import hashlib,json,shutil,sys
import numpy as np
import pytest
HERE=Path(__file__).resolve().parent
ROOT=HERE.parents[2]
SPEC=spec_from_file_location('_i58_runner_contracts',HERE/'run_case.py')
RUNNER=module_from_spec(SPEC);sys.modules[SPEC.name]=RUNNER;SPEC.loader.exec_module(RUNNER)
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()


@pytest.fixture
def workspace(tmp_path,monkeypatch):
 root=tmp_path/'repo';out=root/'results/new';out.mkdir(parents=True)
 seed=root/'generated/offroad/flat/control_schedule.npz';seed.parent.mkdir(parents=True)
 shutil.copyfile(ROOT/'generated/offroad/flat/control_schedule.npz',seed)
 native=root/'src/native.py';native.parent.mkdir();native.write_text('# frozen\n')
 oldadapter=root/'old_adapter.py';oldadapter.write_text('# unchanged original\n')
 newadapter=root/'new_adapter.py';newadapter.write_text('# replacement policy adapter\n')
 plugin=root/'bridge.so';plugin.write_bytes(b'never loaded fake binary')
 historical=root/'historical.npz';historical.write_bytes(b'never integrated fake historical input')
 parameters=dict(mode='hardware',rolling_allocation='least_squares',track_contact_point=False)
 settings=json.loads((ROOT/'results/constrained_handoff_20261126/ablation/protocol.json').read_text())['candidate_settings']
 original=dict(parameters=parameters,candidate_settings=settings,candidate_adapter=dict(path='old_adapter.py',sha256=sha(oldadapter)))
 old=root/'old_protocol.json';old.write_text(json.dumps(original))
 case=dict(case_id='m100_B',variant='native',terrain='marsyard2021',phase_rad=-1.,yaw_deg=45.,duration_s=44.,path=dict(kind='mars_route',route='plain_figure8'))
 protocol=dict(iteration='I-58',cases=[case],parameters=parameters,source_sha256={'src/native.py':sha(native)},
  input_sha256={'old_protocol.json':sha(old),'historical.npz':sha(historical)},
  bridge_binary_sha256=sha(plugin),gain_cache=dict(path=str(seed.relative_to(root)),sha256=sha(seed)),
  frozen_A_protocol_path='old_protocol.json',historical_baseline_paths={'m100_B':'historical.npz'},
  adapters={name:dict(path=adapter.name,sha256=sha(adapter),settings=json.loads(json.dumps(settings)))
   for name,adapter in [('hard_slew',oldadapter),('balance_priority',newadapter)]})
 (out/'protocol.json').write_text(json.dumps(protocol))
 monkeypatch.setattr(RUNNER,'ROOT',root)
 monkeypatch.setattr('spherical_wheel.gazebo.bridge.plugin_path',lambda:plugin)
 monkeypatch.setattr('spherical_wheel.offroad_control.FlatDesign.__init__',lambda*a,**k:(_ for _ in()).throw(AssertionError('No design in preflight')))
 monkeypatch.setattr('spherical_wheel.gazebo.experiments.GazeboWorkspace.__init__',lambda*a,**k:(_ for _ in()).throw(AssertionError('No physical workspace in preflight')))
 return root,out,protocol


def write(out,p):(out/'protocol.json').write_text(json.dumps(p))
def tree(root):return {str(p.relative_to(root)):(p.stat().st_size,p.stat().st_mtime_ns,sha(p))for p in root.rglob('*')if p.is_file()}


def test_preflight_readonly_without_models_or_outputs(workspace):
 root,out,_=workspace;before=tree(root);_,job,audit=RUNNER.preflight(out,'m100_B')
 assert audit['passed']and job['variant']=='native';assert before==tree(root);assert not(out/'campaign').exists()


def test_changed_registered_source_refused(workspace):
 root,out,_=workspace;(root/'src/native.py').write_text('# changed')
 with pytest.raises(ValueError,match='source changed'):RUNNER.preflight(out)


def test_changed_original_adapter_refused(workspace):
 root,out,_=workspace;(root/'old_adapter.py').write_text('# changed')
 with pytest.raises(ValueError,match='adapter source changed'):RUNNER.preflight(out)


def test_changed_plugin_refused_without_load(workspace):
 root,out,_=workspace;(root/'bridge.so').write_bytes(b'changed')
 with pytest.raises(ValueError,match='bridge binary changed'):RUNNER.preflight(out)


def test_nonzero_auxiliary_penalty_refused(workspace):
 _,out,p=workspace;p['adapters']['balance_priority']['settings']['allocator']['yaw_weight']=1;write(out,p)
 with pytest.raises(ValueError,match='auxiliary'):RUNNER.preflight(out)


def test_soft_cost_change_between_A_P_refused(workspace):
 _,out,p=workspace;p['adapters']['balance_priority']['settings']['allocator']['change_weight']=.3;write(out,p)
 with pytest.raises(ValueError,match='soft/model/readiness'):RUNNER.preflight(out)


def test_both_soft_costs_cannot_silently_retune_original_A(workspace):
 _,out,p=workspace
 for d in p['adapters'].values():d['settings']['allocator']['change_weight']=.3
 write(out,p)
 with pytest.raises(ValueError,match='frozen I56A'):RUNNER.preflight(out)


def test_missing_historical_binding_refused(workspace):
 _,out,p=workspace;p['historical_baseline_paths']={};write(out,p)
 with pytest.raises(ValueError,match='historical precision'):RUNNER.preflight(out)


def test_duplicate_case_refused(workspace):
 _,out,p=workspace;p['cases'].append(p['cases'][0].copy());write(out,p)
 with pytest.raises(ValueError,match='unique'):RUNNER.preflight(out)


def test_missing_case_refused(workspace):
 root,out,_=workspace;before=tree(root)
 with pytest.raises(ValueError,match='registered case'):RUNNER.preflight(out,'absent')
 assert before==tree(root)


def test_wrong_original_bank_refused(workspace):
 _,out,p=workspace;p['gain_cache']['sha256']='0'*64;write(out,p)
 with pytest.raises(ValueError,match='bank bytes changed'):RUNNER.preflight(out)


def test_bytewise_parity_preserves_signed_zero_dtype_and_nan_payload():
 assert not RUNNER.bit_exact(np.array([0.]),np.array([-0.]))
 assert not RUNNER.bit_exact(np.array([1.]),np.array([1.],np.float32))
 assert RUNNER.bit_exact(np.array([np.nan]),np.array([np.nan]))


def test_simultaneous_reference_or_gain_change_refused(workspace):
 _,out,p=workspace;p['parameters']['schedule_on_ground_normal']=True
 # Use independent dict in fixture: parameters shared with original only in memory,
 # on-disk frozen original remains fixed.
 write(out,p)
 with pytest.raises(ValueError,match='parameters differ'):RUNNER.preflight(out)
