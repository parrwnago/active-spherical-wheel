"""Compact narrative/JSON from sealed I51 reductions; no new physical analysis."""
from datetime import datetime,timezone
from pathlib import Path
import hashlib
import json
import shutil

HERE=Path(__file__).resolve().parent
FAMILY=HERE.parent
ROOT=FAMILY.parents[1]


def sha(path):return hashlib.sha256(path.read_bytes()).hexdigest()
def main():
    targets=[HERE/'summary.json',HERE/'README.md',HERE/'summary_provenance.json']
    if any(path.exists()for path in targets):raise FileExistsError('Preserve existing summary stage')
    source=Path(__file__).resolve();snapshot=HERE/'summary_source_snapshot';snapshot.mkdir(exist_ok=False)
    shutil.copyfile(source,snapshot/source.name)
    paths=[HERE/'stage01/report.json',HERE/'phase_stage01/report.json',FAMILY/'protocol.json',FAMILY/'phase_schedule_isolation/protocol.json']
    paths.extend(FAMILY/'campaign/runs'/case/'summary.json'for case in ('base030','reference030','base060','handoff060'))
    inputs={str(path.relative_to(ROOT)):sha(path)for path in paths}
    (HERE/'summary_provenance.json').write_text(json.dumps(dict(captured_before_transformation_utc=datetime.now(timezone.utc).isoformat(),
        source_sha256=sha(source),input_sha256=inputs,scope='Compact reduction of existing reports only; units/table arithmetic, no controller/model/physics or metricdefinition changes.'),indent=2)+'\n')
    report=json.loads((HERE/'stage01/report.json').read_text());phase=json.loads((HERE/'phase_stage01/report.json').read_text())
    cases={}
    for case,row in report['cases'].items():
        raw_summary=json.loads((FAMILY/'campaign/runs'/case/'summary.json').read_text())
        cases[case]=dict(completed=row['completed'],cap_m_s=row['cap_m_s'],moving_rms_mm=1000*row['timed_tracking']['moving_rms_m'],
            peak_mm=1000*row['timed_tracking']['full_peak_m'],final_mm=1000*row['timed_tracking']['final_m'],
            peak_reference_lean_deg=row['attitude']['peak_reference_error_deg'],peak_absolute_lean_deg=row['attitude']['peak_absolute_deg'],
            settled=row['original_settling']['passed'],old_route_gates_passed=row['original_route_gates']['passed'],
            first20mm_time_s=None if row['departure']['first_sustained_20mm']is None else row['departure']['first_sustained_20mm']['start_s'],
            max_actual_joint_speeds_rad_s=raw_summary['actuation']['max_joint_speed_rad_s'])
    comparisons={candidate:dict(baseline_id=info['baseline_id'],moving_RMS_reduction_percent=100*info['moving_RMS_relative_reduction'],
        passes_all_conditions=info['passes_all_conditions'],failed_checks=[key for key,value in info['checks'].items()if not value],
        turn_changes=info['turn_changes'])for candidate,info in report['comparisons'].items()}
    compact=dict(iteration='I-51',status='Five declared native runs completed; all evidence retained; no default adoption.',
        sources=dict(primary_report_sha256=sha(HERE/'stage01/report.json'),phase_report_sha256=sha(HERE/'phase_stage01/report.json')),
        primary=dict(cases=cases,paired_comparisons=comparisons,
            reference_mechanism=report['cases']['reference030']['mechanistic'],handoff_mechanism=report['cases']['handoff060']['mechanistic']),
        phase_addendum=dict(prefix_fields=len(phase['prefix']['all_common_precision_fields_byte_identical']),prefix_samples=phase['prefix']['samples'],first_gain_difference_s=phase['first_gain_difference_s'],cases=phase['cases']),
        conclusions=[
            'Original leastsquares doesnot intentionallyrelax position orretime thepath. Referenceguard preserves thosegoals butacceptsnominalrollingmismatch.',
            'Referencecandidate improvesmovingRMS/peak/final/lean butfails turns/settling. Itsnativejointreferences remainbelow20, so thishistory onlybinds thereferenceaccelerationguard.',
            'Handoffcandidate improvesRMS/peak/final butworsenspeakrelativelean andseveralturns. Episodebelt slew isbounded; nativejointreference singularities arenotgloballyfixed.',
            'Latephase scheduling changesafteranexact38shistory reduce late/finalerror butnotwholemovingRMS orsettling. Freeheadingyawchange canworsen.',
            'No combination, outcome-basedretuning orbroadertrajectorycampaign wasperformed; defaults remainnative.'])
    (HERE/'summary.json').write_text(json.dumps(compact,indent=2)+'\n')
    rows=['| Arm | Moving RMS mm | Peak mm | Final mm | Relative lean max ° | Settled / old gates |',
          '|---|---:|---:|---:|---:|---|']
    for case,row in cases.items():rows.append(f"| {case} | {row['moving_rms_mm']:.3f} | {row['peak_mm']:.3f} | {row['final_mm']:.3f} | {row['peak_reference_lean_deg']:.3f} | {row['settled']} / {row['old_route_gates_passed']} |")
    text='''# I51 — paired mechanism tests and late phase diagnostic

All four primary44s runs and the separately registered fifth44s diagnostic complete without falling. The0.30/0.60 baselines reproduce every common I49 precision field exactly. Initial states, original time/XY/V/A program and fixed gain-bank/ordinary design match. No runtime/config/default files changed.

The original least-squares controller does not deliberately move the path target or relax its timing. The reference candidate keeps those targets and permits internal rolling-reference mismatch to avoid extreme joint-reference changes. It is not a path-following governor or a guarantee of total-command feasibility.

## Primary matched-cap comparisons

'''+ '\n'.join(rows)+'''

Compare `reference030` only with `base030`, and `handoff060` only with `base060`. Both improve moving RMS by more than the registered10% threshold, but both fail the complete multiobjective screen. Favorable peak/final reductions remain useful evidence despite those failures.

The reference arm reduces moving RMS12.43%, peak/final errors and relative lean. Its references are modified on53 held samples(265ms), beginning6.700s. Native reference maximum5.380rad/s never reaches the20rad/s ceiling: the observed intervention is reference-slew limiting and its subsequent changed state history. It is not a same-history demonstration of directly clipping the baseline31.650s singular request. Guarded maximum is3.119rad/s and reference acceleration100rad/s². Modeled introduced rolling residual reaches0.09005m/s on modified rows; it is not measured tracking error or dynamic feasibility. Turn-four overshoot worsens by11.468mm; settling and old route gates fail.

The handoff arm reduces moving RMS19.92% and final error49.231→15.575mm. It changes496 held samples(2.480s), also beginning6.700s, and respects3m/s² during active episodes. Peak reference-relative lean worsens6.053→11.683°, and several turns regress. Native joint references still reach143.914rad/s; this intervention does not solve reference singularities. Raw native box clipping and additional handoff projection are reported separately. The common clipping delta is total actual-minus-raw, not solely actuator saturation.

All six fixed turn reversals remain covered. Signed-lag changes and absolute timing error have different meanings: reference turn3 changes−0.469→−0.183s and turn4−0.444→+0.374s, both closer to zero in absolute terms while failing the registered signed-increase guard. Reference turn6 instead worsens+0.569→+0.903s. No unfavorable turn is discarded.

## Separately registered late phase scheduling

The fifth case changes only world-Z versus sensed-normal gain scheduling from38s. All47 common precision fields match the fresh0.30 baseline for7,600 preactivation observations; state/reference at38s match and gain differences begin at38s.

| Metric | base030 | phase_late030 |
|---|---:|---:|
| Timed RMS38–42s |57.857mm|48.323mm|
| Timed RMS38–44s |84.652mm|56.707mm|
| Full moving RMS |50.486mm|50.495mm|
| Full peak error |190.901mm|96.309mm|
| Final error44s |190.191mm|53.993mm|
| Full peak relative lean |8.709°|6.089°|

This matched-history intervention supports a contribution from gain scheduling in this late episode. It does not validate globally changing the phase definition: earlier global normal-relative scheduling failures remain relevant. It remains unsettled and fails the original route gates. Free-heading yaw change over38–44s grows in magnitude from−109.755° to−142.269°, so improved position does not imply uniformly better behavior. Joint tracking RMS in the addendum is a three-element per-joint vector, not the aggregate vector-norm RMS used in some earlier audits.

No combined candidate, outcome-based parameter tuning or broader trajectory expansion was performed. The native defaults remain unchanged. Reference/episode bounds are mechanistic successes; tracking, balance, friction and actual motor capability remain separate questions.

[Primary report](stage01/report.json) · [Turn table](stage01/turn_metrics.csv) · [Tracking plot](stage01/paired_tracking.png) · [Turn plot](stage01/paired_turns.png) · [Separate phase addendum](phase_stage01/report.json) · [Late error/yaw plot](phase_stage01/late_phase_comparison.png) · [Compact JSON](summary.json).
'''
    (HERE/'README.md').write_text(text)
    for relative,expected in inputs.items():assert sha(ROOT/relative)==expected
    (HERE/'summary_artifact_manifest.json').write_text(json.dumps(dict(files=[dict(path=str(p.relative_to(HERE)),bytes=p.stat().st_size,sha256=sha(p))for p in targets+[snapshot/source.name]]),indent=2)+'\n')
    print('summary_sha256',sha(HERE/'summary.json'))


if __name__=='__main__':main()
