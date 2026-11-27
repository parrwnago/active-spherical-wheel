"""Register I68 capture before any new physics; immutable output path."""
from datetime import datetime, timezone
from pathlib import Path
import hashlib
import json
import subprocess

ROOT = Path(__file__).resolve().parents[3]
BASE = Path(__file__).resolve().parents[1]


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def main():
    parent_path = ROOT/'results/balance_priority_20261126/protocol.json'
    parent = json.loads(parent_path.read_text())
    sources = dict(parent['source_sha256'])
    for path in sorted((BASE/'campaign_tools').glob('*.py')):
        sources[str(path.relative_to(ROOT))] = sha(path)
    for path in sorted((BASE/'instrumentation').glob('*.py')):
        sources[str(path.relative_to(ROOT))] = sha(path)
    design = BASE/'analysis/design_review.md'
    sources[str(design.relative_to(ROOT))] = sha(design)
    assert len(list((BASE/'instrumentation').glob('test_*.py'))) >= 1
    assert (BASE/'instrumentation/substep_trace.py').is_file()
    for relative, expected in sources.items():
        assert sha(ROOT/relative) == expected, relative
    inputs = {key: value for key, value in parent['input_sha256'].items() if not key.endswith('.so')}
    for path in (parent_path, ROOT/'results/balance_priority_20261126/campaign/runs/m100_B/precision.npz',
                 ROOT/parent['gain_cache']['path']):
        inputs[str(path.relative_to(ROOT))] = sha(path)
    binaries = {key: value for key, value in parent['input_sha256'].items() if key.endswith('.so')}
    protocol = dict(iteration='I-68', registered_utc=datetime.now(timezone.utc).isoformat(),
        base_commit=subprocess.check_output(['git','rev-parse','HEAD'], cwd=ROOT, text=True).strip(),
        status='Registered before the new replay; instrumentation only, no controller improvement claimed.',
        question='Are endpoint contact/load-routing assumptions inconsistent with the preceding 5 ms, or does substantial observer/model residual remain even with resolved solver-contact forces?',
        case=dict(case_id='m100_B', variant='native', engine='Gazebo Harmonic 8.10.0 / DART 6.19.5 / gz-physics 7.5.0 / ODE 0.16.6',
            terrain='marsyard2021_dem', route='plain_figure8', phase_rad=-1., yaw_deg=45., duration_s=44.,
            physics_dt_s=.001, control_dt_s=.005, capture_hold_start_window_s=[0.,44.],
            input_bounds=[1.5,.6,.6,.3,.3], parameters=parent['parameters']),
        unchanged=['Original I58 runner imported in its original location',
            'All native startup design/reset and full trajectory history',
            'One controller/observer update and held command per 5 ms',
            'Live LQR, momentum observer, geometry locator, rolling/holding feedforward and 120 ms incoming preparation',
            'Compiled Gazebo bridge and exact-physics plugins; no rebuild'],
        change='Inside physical.step only, replace native run(5) with five run(1); read-copy bridge state and all contact rows before/after. No mid-hold mirror update, controller call or command.',
        evaluation_only='All solver contact rows/forces/normals and geometry-based slip calculations. They never enter native control or estimator selection.',
        data_coverage_limits=['Bridge exports point force, not contact couples or every possible generalized constraint force.',
            'Generated models snapshotted after native construction/ground checks/resets and before control, not before setup physics.',
            'One previously inspected starting phase only; no unseen validation or demonstrated controller gain.'],
        validity_checks=['All shared full-horizon I58 precision arrays byte-exact in shape/dtype/data',
            '8800 completed holds / 44000 one-ms samples, no failed hold or mid-hold reset',
            'Actual held input and disturbance unchanged through all five solves',
            'Finite state/contact fields and known collision tags; retain rejected rows and diagnose their force',
            'Endpoint states and commands match precision joins; native bridge clock distinct from nominal controller clock'],
        analysis=dict(source_freeze='Design and numerical comparisons predeclared here; analysis source/input seal before data arithmetic.',
            groups=dict(primary=[1.,41.], primary_right_open=True, broad=[32.,36.], fixed_event_half_window_s=.100,
                fixed_event_times_s=[7.875,32.110,35.055,35.205]),
            frame='Sum force-mapped native generalized impulses at each pre-solve pose; average native components, then rotate only angular rows with exact observer interval-midpoint rotation.',
            geometry_control='Repeat identical force mapping at post-solve poses; preserve separately, not an uncertainty interval.',
            loads='Compare raw observer interval_mean9 with five-step force-mapped reference and last-step reference, grouped into forceN/base momentNm/jointNm; no mixed-unit norm.',
            decomposition='mu - inferred_endpoint = (mu - interval_proxy) + (interval_proxy - inferred_endpoint); retain vector cross term and avoid additive causal percentages.',
            routing_criterion='Joint-vector RMS(mu - integrated) strictly below RMS(mu - last_step) in BOTH pre/post pose controls for the prespecified group; report continuous values even on failure. This is interval-consistency evidence, not calibrated agreement or CPU latency.',
            slip='Contact-normal tangential material velocity using actual q/v, plus projected carrier-y belt surface speed; pressure-weighted vector mean norm, RMS speed and max positive-load speed, pre/post controls.',
            normals='Compare actual pressure-weighted surface normal and dispersion with intrinsic-normal hint and native filtered normal. Continuous magnitudes, no newly fitted slip/normal threshold.',
            thresholds='No threshold/weight/calibration changes; old affine compatibility is historical only.',
            stop='If replay diverges, preserve it and diagnose parity before interpreting original event times. No source/scorer/controller change from this experiment alone.'),
        source_sha256=sources, input_sha256=inputs, binary_sha256=binaries,
        reproduction='Run campaign_tools/launch_replay.py in a fresh process from repository root; never overwrite existing campaign output.')
    path = BASE/'protocol.json'
    with path.open('x') as stream:
        json.dump(protocol, stream, indent=2, allow_nan=False)
        stream.write('\n')
    print(json.dumps(dict(path=str(path), sha256=sha(path), source_files=len(sources), input_files=len(inputs), binaries=len(binaries))))


if __name__ == '__main__':
    main()
