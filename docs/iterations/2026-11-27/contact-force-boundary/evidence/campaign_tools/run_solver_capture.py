"""I69 private delegate-hook replay of unchanged I58 native history."""
from contextlib import ExitStack
from datetime import datetime,timezone
from pathlib import Path
from unittest.mock import patch
import argparse
import ctypes
import hashlib
import importlib.util
import json
import shutil
import traceback
import numpy as np

ROOT=Path(__file__).resolve().parents[3]
BASE=Path(__file__).resolve().parents[1]
OLD=ROOT/'results/contact_interval_replay_20261127'
PARENT=ROOT/'results/balance_priority_20261126'
TARGETS={1575,6422,7011,7041}


def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def save(p,v):Path(p).write_text(json.dumps(v,indent=2,allow_nan=False)+'\n')
def module(path,name):
    spec=importlib.util.spec_from_file_location(name,path);m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m);return m


def main():
    ap=argparse.ArgumentParser();ap.add_argument('--run',action='store_true');args=ap.parse_args()
    if not args.run:raise SystemExit('Explicit --run required for registered new solver observation')
    protocol=json.loads((BASE/'phase_B_protocol.json').read_text())
    for group in ('sources','inputs','binaries'):
        for rel,h in protocol[group].items():assert sha(ROOT/rel)==h,(group,rel)
    out=BASE/'campaign/native_solver_capture';out.mkdir(parents=True,exist_ok=False)
    shutil.copyfile(PARENT/'protocol.json',out/'protocol.json')
    shutil.copyfile(BASE/'phase_B_protocol.json',out/'phase_B_protocol.json')
    for rel in protocol['sources']:
        target=out/'source_snapshot'/rel;target.parent.mkdir(parents=True,exist_ok=True);shutil.copyfile(ROOT/rel,target)
    parent=module(PARENT/'campaign_tools/run_case.py','_i69_original_i58')
    previous=module(OLD/'campaign_tools/run_interval_replay.py','_i69_i68_parity_helper')
    collector=module(OLD/'instrumentation/substep_trace.py','_i69_unchanged_i68_collector')
    parent.preflight(out,'m100_B')
    library=ROOT/protocol['probe_library']
    probe=ctypes.CDLL(str(library),mode=ctypes.RTLD_GLOBAL)
    probe.asw_i69_probe_context.argtypes=[ctypes.c_double,ctypes.c_int,ctypes.c_int]
    probe.asw_i69_probe_counts.argtypes=[ctypes.POINTER(ctypes.c_uint64)]
    probe.asw_i69_probe_context(0.,0,0)
    import spherical_wheel.gazebo.experiments as gx
    native_plant=gx.GazeboWorkspace.plant;traces=[];context={'hold':False};success=False
    with ExitStack() as stack:
        def plant(workspace,terrain):
            physical=native_plant(workspace,terrain)
            if traces:return physical
            native_run=physical.sim.run
            def observed_run(iterations):
                if context['hold']:
                    assert iterations==1,'Collector must split each native hold into five solves'
                    context['substep']+=1
                    probe.asw_i69_probe_context(context['start'],context['substep'],int(context['end_index'] in TARGETS))
                else:probe.asw_i69_probe_context(0.,0,0)
                return native_run(iterations)
            stack.enter_context(patch.object(physical.sim,'run',observed_run))
            trace=collector.SubstepTrace(physical,start_s=0.,end_s=44.,physics_dt_s=.001)
            stack.enter_context(trace.instrument());traces.append(trace)
            trace_step=physical.step
            def observed_step(command,disturbance=None):
                assert not context['hold']
                context.update(hold=True,start=float(physical.data.time),substep=0,
                    end_index=int(round(float(physical.data.time)/physical.dt))+1)
                try:
                    result=trace_step(command,disturbance)
                    assert context['substep']==5
                    return result
                finally:
                    probe.asw_i69_probe_context(0.,0,0);context['hold']=False
            stack.enter_context(patch.object(physical,'step',observed_step))
            return physical
        try:
            with patch.object(gx.GazeboWorkspace,'plant',plant):parent.run_case(out,'m100_B')
            success=True
        except BaseException:
            save(out/'failure.json',dict(traceback=traceback.format_exc()));raise
        finally:
            probe.asw_i69_probe_context(0.,0,0)
            counts=(ctypes.c_uint64*3)();probe.asw_i69_probe_counts(counts)
            saved=[trace.save(out/'substeps.npz') for trace in traces]
            save(out/'capture_receipt.json',dict(iteration='I-69',success=success,
                completed_utc=datetime.now(timezone.utc).isoformat(),counts=dict(zip(['Dantzig','PGS','applyImpulse'],list(counts))),
                trace=saved,measurement_boundary='Solver/contact hooks copy inputs/output and delegate installed functions unchanged. No new commands or estimator updates.'))
    comparison=previous.compare_precision(out/'campaign/runs/m100_B/precision.npz',PARENT/'campaign/runs/m100_B/precision.npz')
    save(out/'i58_precision_comparison.json',comparison)
    assert comparison['all_shared_fields_byte_exact'] and comparison['shared_fields']==59
    assert not comparison['old_only'] and not comparison['new_only']
    with np.load(out/'substeps.npz',allow_pickle=False) as new,np.load(OLD/'campaign/native_substeps/substeps.npz',allow_pickle=False) as old:
        arrays={key:dict(exact=new[key].dtype==old[key].dtype and new[key].shape==old[key].shape and new[key].tobytes()==old[key].tobytes())
            for key in old.files}
    save(out/'i68_trace_comparison.json',dict(fields=arrays,all_fields_byte_exact=all(r['exact'] for r in arrays.values())))
    assert all(r['exact'] for key,r in arrays.items() if key!='metadata_json'),'Physics substeps changed under logging'
    for group in ('sources','inputs','binaries'):
        for rel,h in protocol[group].items():assert sha(ROOT/rel)==h,(group,rel)
    assert (out/'solver_packets.bin').stat().st_size>0,'No installed method interposition captured'
    print(json.dumps(dict(success=success,precision_fields=59,all_native_precision_exact=True,
        trace_arrays_exact=sum(r['exact'] for r in arrays.values()),trace_fields=len(arrays))),flush=True)


if __name__=='__main__':main()
