"""Freeze I58 conditions and gates before native execution."""
from datetime import datetime, timezone
from pathlib import Path
import hashlib
import json
import subprocess

ROOT = Path.cwd()
BASE = ROOT/'results/balance_priority_20261126'
OLD = ROOT/'results/constrained_handoff_20261126'
sha = lambda p: hashlib.sha256(Path(p).read_bytes()).hexdigest()
if (BASE/'protocol.json').exists():
    raise FileExistsError('Preserve an earlier registered protocol')
parent = json.loads((OLD/'protocol.json').read_text())
ablation_path = OLD/'ablation/protocol.json'
ablation = json.loads(ablation_path.read_text())
settings = ablation['candidate_settings']
assert settings['allocator']['yaw_weight'] == settings['allocator']['momentum_weight'] == 0
cases = []
for phase, angle, variants in [('m100', -1., ('B','A','P')), ('p030', .3, ('B','P')), ('p090', .9, ('B','P'))]:
    for letter in variants:
        cases.append(dict(case_id=phase+'_'+letter,
            variant={'B':'native','A':'hard_slew','P':'balance_priority'}[letter],
            terrain='marsyard2021', phase_rad=angle, yaw_deg=45., duration_s=44.,
            path=dict(kind='mars_route', route='plain_figure8')))
historical = dict(m100_B=str((OLD/'campaign/runs/m100_B/precision.npz').relative_to(ROOT)),
    m100_A=str((OLD/'ablation/campaign/runs/m100_A/precision.npz').relative_to(ROOT)),
    p030_B=parent['historical_baseline_paths']['p030_B'],
    p090_B=parent['historical_baseline_paths']['p090_B'])
paths = [*sorted((ROOT/'src/spherical_wheel').rglob('*.py')),
    *sorted((ROOT/'config').glob('*.yaml')),
    ROOT/'gazebo/plugin/SphericalWheelBridge.cc', ROOT/'scripts/run-lqr-contact-forecast.py',
    BASE/'campaign_tools/run_case.py', BASE/'campaign_tools/adapter.py',
    BASE/'analysis/metrics.py', Path(__file__).resolve(),
    OLD/'campaign_tools/adapter.py',
    ROOT/'tests/test_handoff_control.py', ROOT/'tests/test_handoff_model.py',
    ROOT/'tests/test_balance_priority_control.py']
assert all(p.exists() for p in paths)
inputs = [*map(lambda p: ROOT/p, historical.values()), ablation_path,
          *[ROOT/p for p in parent['input_sha256'] if p.endswith('.so')]]
adapters = dict(hard_slew=dict(path=ablation['candidate_adapter']['path'],
    sha256=ablation['candidate_adapter']['sha256'], settings=settings),
    balance_priority=dict(path=str((BASE/'campaign_tools/adapter.py').relative_to(ROOT)),
    sha256=sha(BASE/'campaign_tools/adapter.py'), settings=settings))
acceptance = dict(
    completion='Observed44s without failure; no interrupted case treated as complete.',
    moving_window_s=[1.,41.], yaw_window_s=[28.,44.],
    balance_response_tolerance=1e-8, priority_RMS_ratio_vs_A_max=.1,
    old_A_priority_peak_min_rad_s=.01,
    primary_benefit='TOTAL G20[3:5]*(final-clipped_native) max<=1e-8 at every emission; held RMS<=10%oldA, oldApeak>=.01. Algorithmic/local along-trace restoration, not same-state physical stability proof.',
    continuity='SECONDARY ONLY: caps-normalized actual step L2 RMS,p99,max; first actualprior included, final unheld excluded. Not an alternative benefit selected after results.',
    timed_xy_rms_ratio_max=1.1, timed_xy_rms_add_m=.001,
    timed_xy_peak_add_m=.01, final_xy_add_m=.01,
    relative_lean_peak_add_deg=1., relative_lean_rms_ratio_max=1.1,
    relative_lean_rms_add_deg=.1, absolute_tilt_peak_add_deg=1.,
    each_turn_overshoot_add_m=.01,
    yaw_peak_abs_ratio_max=1.05, yaw_peak_abs_add_rad_s=.01,
    heading_absolute_travel_ratio_max=1.05, heading_absolute_travel_add_rad=.01,
    planar_speed_peak_ratio_max=1.1, planar_speed_peak_add_m_s=.01,
    linear_acceleration50ms_peak_ratio_max=1.1, linear_acceleration50ms_peak_add_m_s2=.1,
    acceleration_definition='Full0..44s peak norm of all3worldlinear velocity backward differences over10recordedsteps/actual50ms. Endpoint-average derivative, not instantaneous solver acceleration.',
    settling='IfBpasses existing rawtargetsettling gate Pmustalsopass; alwaysreportboth.',
    engineering_scope='Fixed engineering nonregression tolerances, no significance or stability claim.',
    all_required=True,
    broader_rule='Onlyifm100PpassesALLtechnical,primarybenefitandmotiongatesrunp030B/P,p090B/P. Stopbroaderonleadfailure; no postoutcomeretuning/fallbackcandidate.',
    adoption='No automatic default promotion evenif all three phase screens pass.')
protocol = dict(iteration='I-58', registered_utc=datetime.now(timezone.utc).isoformat(),
    base_commit=subprocess.check_output(['git','rev-parse','HEAD'],text=True).strip(),
    question='Preserve clipped-native transverse response inside actual amplitude caps; remove only I56A hard slew feasibility, retaining identical soft costs and disabled yaw/momentum.',
    cases=cases, parameters=ablation['parameters'],
    source_sha256={str(p.relative_to(ROOT)):sha(p) for p in paths},
    input_sha256={str(p.relative_to(ROOT)):sha(p) for p in inputs},
    gain_cache=parent['gain_cache'], bridge_binary_sha256=parent['bridge_binary_sha256'],
    frozen_A_protocol_path=str(ablation_path.relative_to(ROOT)), adapters=adapters,
    historical_baseline_paths=historical, require_historical_baseline_exact=True,
    information_boundary='Exact simulated body/joints and actualpriorcommand; contacts from momentumobserver+geometry only. Truthcontacts evaluationonly. Rawtargets/nativegainbank/previewunchanged.',
    algorithm=dict(hard_constraints='Total amplitude caps and clipped-native Gbodyxy response equality',
        optional_smoothing='Exact I56A cap-normalized softchange cost; original60Nm/s,6m/s2 comparisons diagnostic only forP',
        secondary_cost='Same travelresponse/native+readiness/change/effort terms and weights asA',
        auxiliary_yaw_momentum='Both weights strictlyzero; no velocitybaseline change',
        model='Same frozen I56 G20 softcontact strength10, mismatchconfidence scale.02m/s. Not actual balance/friction/contact prediction proof.'),
    acceptance=acceptance, initial_execution_order=['m100_B','m100_A','m100_P'],
    conditional_execution_order=['p030_B','p030_P','p090_B','p090_P'],
    max_fresh_native_runs=7, recorded_before_new_native_outcomes=True)
(BASE/'protocol.json').write_text(json.dumps(protocol,indent=2)+'\n')
print(json.dumps(dict(protocol_sha256=sha(BASE/'protocol.json'), cases=len(cases),sources=len(paths),inputs=len(inputs))))
