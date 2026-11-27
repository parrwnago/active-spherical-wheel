"""I72 geometry-first selector evaluation, after all inference outputs seal.

Primary C is newly accepted fused physical candidate, otherwise native N's
family and equivalent point.  No new inference, fitting or physics occurs here.
"""
from pathlib import Path
from datetime import datetime,timezone
import argparse,importlib.util,json,shutil
import numpy as np

REFERENCE=Path(__file__).with_name('reference_i70')/'evaluate_temporal.py'
spec=importlib.util.spec_from_file_location('i72_reference_metrics',REFERENCE)
ref=importlib.util.module_from_spec(spec);spec.loader.exec_module(ref)
REFERENCE_SHA='33993c7fc892d71a4530666090af957cb77614543b03fe3b4316bfdc8b351008'
CASES=('m100_B','p030_B','p090_B')
METHODS=('N','oldF','A','newF','newVG','C','VG_C')
DEFAULT_EVENTS=(('m100_B',7.875),('m100_B',32.110),('m100_B',35.055),('m100_B',35.205),('p030_B',13.470))


def policies(packet):
    output={}
    for m in ('N','oldF','A','newF','newVG'):
        output[m]=dict(family=packet[m+'_family'].copy(),point_m=packet[m+'_point_m'].copy())
    for m,source in [('C','newF'),('VG_C','newVG')]:
        accepted=packet[source+'_accepted']
        output[m]=dict(family=np.where(accepted,packet[source+'_family'],packet['N_family']),
            point_m=np.where(accepted[:,None],packet[source+'_point_m'],packet['N_point_m']))
    return output


def point_report(primary,target,methods):
    target_valid=np.isfinite(target).all(axis=1)
    distance,valid={},{}
    report={}
    for m,value in methods.items():
        p=value['point_m'];valid[m]=(value['family']>=0)&np.isfinite(p).all(axis=1)&target_valid
        distance[m]=np.linalg.norm(p-target,axis=1)*1000.
        distance[m][~valid[m]]=np.nan
        mask=primary&valid[m]
        report[m]=dict(denominator=int(primary.sum()),finite=int(mask.sum()),missing=int(np.sum(primary&~valid[m])),
            coverage=ref.fraction(int(mask.sum()),int(primary.sum())),all_finite_primary_mm=ref.summary(distance[m][mask]))
    fixed=primary&valid['N']
    complete=bool(valid['C'][fixed].all())
    paired=dict(reference_count=int(fixed.sum()),C_missing_on_native_reference=int(np.sum(fixed&~valid['C'])),
        C_complete_on_native_reference=complete,N_mm=ref.summary(distance['N'][fixed]),
        C_mm=ref.summary(distance['C'][fixed]) if complete else ref.summary([]),
        C_finite_part_mm=ref.summary(distance['C'][fixed&valid['C']]),
        reference='Fixedprimary rows with finite dominant-familyrawcentroid and N equivalent point; wrongfamilies retained.')
    return dict(methods=report,fixed_primary_C_vs_N=paired),distance,valid


def cohort_report(frame):
    primary=frame['primary_dominant']
    Nc,Ac=frame['N__correct_family'],frame['A__correct_family']
    groups=dict(old_useful=primary&~Nc&Ac,old_harmful=primary&Nc&~Ac,
                old_accepted_wrong=primary&frame['old_accepted']&~Ac)
    result={}
    for name,mask in groups.items():
        result[name]=dict(denominator=int(mask.sum()),methods={m:dict(correct=int(np.sum(mask&frame[m+'__correct_family'])),
            wrong=int(np.sum(mask&~frame[m+'__correct_family']))) for m in METHODS})
    for m in ('newF','newVG'):
        accepted=primary&frame[m+'_accepted']
        result[m+'_accepted']=dict(denominator=int(accepted.sum()),wrong=int(np.sum(accepted&~frame[m+'__correct_family'])),
            native_wrong_on_same_rows=int(np.sum(accepted&~Nc)))
    result['C_vs_A']=dict(denominator=int(primary.sum()),
        gained=int(np.sum(primary&~Ac&frame['C__correct_family'])),lost=int(np.sum(primary&Ac&~frame['C__correct_family'])))
    return result


def coverage(mask,old_accept,new_accept):
    d=int(mask.sum());old=int(np.sum(mask&old_accept));new=int(np.sum(mask&new_accept))
    return dict(denominator=d,old_A_accepted=old,new_fused_accepted=new,
        old_A_fraction=ref.fraction(old,d),new_fused_fraction=ref.fraction(new,d),
        fixed_domain=True)


def evaluate_case(packet):
    time=packet['time_s'];truth=ref.truth_labels(packet['truth_patch_loads_N'])
    window=ref.primary_window(time);primary=window&truth['truth_dominant']
    if not np.array_equal(primary,packet['old_primary_dominant']) or not np.array_equal(truth['truth_family'],packet['old_truth_family']):
        raise ValueError('Newevaluation changes frozenI60 primarylabels/domain')
    methods=policies(packet);row=np.arange(len(time));target=packet['truth_patch_points_m'][row,np.maximum(truth['truth_family'],0)]
    reports={};frame=dict(time_s=time,primary_dominant=primary,primary_window=window,
        old_ambiguous_motion=packet['old_ambiguous_motion'],old_accepted=packet['old_accepted'],
        newF_accepted=packet['newF_accepted'],newVG_accepted=packet['newVG_accepted'],
        reference_dominant_point_m=target,**truth)
    for m,value in methods.items():
        bits=ref.family_bits(value['family'])
        report,arrays=ref.method_metrics(bits,ref.onehot(bits),truth,window)
        reports[m]=dict(singleton=report['singleton'],loaded_support=report['multiset'])
        frame[m+'__family']=value['family'];frame[m+'__point_m']=value['point_m'];frame[m+'__bits']=bits
        frame[m+'__correct_family']=arrays['correct_family'];frame[m+'__correct_set']=arrays['correct_set']
    points,distance,valid=point_report(primary,target,methods)
    for m in METHODS:frame[m+'__point_distance_mm']=distance[m];frame[m+'__point_valid']=valid[m]
    for key,value in packet.items():
        if key.startswith(('newF__','newVG__')) or key in ('anchor_available','selected_candidate_id','selected_row_index','family_distance_squared_m2'):
            frame[key]=value
    timing=ref.timing(time,truth['truth_family'],truth['truth_dominant'],{m:frame[m+'__bits'] for m in METHODS})
    acceptance=dict(old_ambiguous_motion=coverage(packet['old_ambiguous_motion'],packet['old_accepted'],packet['newF_accepted']),
        all_primary=coverage(primary,packet['old_accepted'],packet['newF_accepted']))
    accepted_only={}
    for source in ('newF','newVG'):
        mask=primary&packet[source+'_accepted']
        accepted_only[source]=dict(denominator=int(mask.sum()),wrong_family=int(np.sum(mask&~frame[source+'__correct_family'])),
            point_mm=ref.summary(distance[source][mask&valid[source]]),finite_point_count=int(np.sum(mask&valid[source])),
            native_on_same_rows_mm=ref.summary(distance['N'][mask&valid['N']]),
            qualification='Secondary accepted-only subset; not substituted for fullprimary C gates.')
    return frame,dict(counts=dict(primary_window=int(window.sum()),primary_dominant=int(primary.sum()),
        loaded=int(np.sum(window&truth['truth_loaded'])),mixed_pressure=int(np.sum(window&truth['truth_loaded']&~truth['truth_dominant']))),
        methods=reports,acceptance=acceptance,points=points,accepted_only=accepted_only,
        cohorts=cohort_report(frame),timing=timing)


def aggregate(frames,cases):
    f={key:np.concatenate([frames[c][key] for c in CASES]) for key in frames[CASES[0]]}
    truth={k:v for k,v in f.items() if k.startswith('truth_')};window=f['primary_window'];primary=f['primary_dominant']
    methods={m:dict(family=f[m+'__family'],point_m=f[m+'__point_m']) for m in METHODS}
    target=f['reference_dominant_point_m']
    points,distances,valid=point_report(primary,target,methods)
    report={}
    for m in METHODS:
        metrics,_=ref.method_metrics(f[m+'__bits'],ref.onehot(f[m+'__bits']),truth,window)
        report[m]=dict(singleton=metrics['singleton'],loaded_support=metrics['multiset'])
    timing={}
    for m in METHODS:
        offsets=[e['methods'][m]['signed_latency_s'] for c in CASES for e in cases[c]['timing']['events']
                 if not e['methods'][m]['missed']]
        timing[m]={k:sum(cases[c]['timing']['methods'][m][k] for c in CASES) for k in
            ('events','matched','missed','reverted_attempts','false_switches','false_singleton_toggles','false_unknown_transitions','false_pair_transitions')}
        timing[m].update(signed_latency_s=ref.summary(offsets),absolute_latency_s=ref.summary(np.abs(offsets)))
    accepted_only={}
    for source in ('newF','newVG'):
        mask=primary&f[source+'_accepted']
        accepted_only[source]=dict(denominator=int(mask.sum()),wrong_family=int(np.sum(mask&~f[source+'__correct_family'])),
            point_mm=ref.summary(distances[source][mask&valid[source]]),finite_point_count=int(np.sum(mask&valid[source])),
            native_on_same_rows_mm=ref.summary(distances['N'][mask&valid['N']]))
    return dict(counts=dict(primary_window=int(window.sum()),primary_dominant=int(primary.sum()),
        loaded=int(np.sum(window&truth['truth_loaded'])),mixed_pressure=int(np.sum(window&truth['truth_loaded']&~truth['truth_dominant']))),
        methods=report,points=points,accepted_only=accepted_only,cohorts=cohort_report(f),timing=timing,
        acceptance=dict(old_ambiguous_motion=coverage(f['old_ambiguous_motion'],f['old_accepted'],f['newF_accepted']),
                        all_primary=coverage(primary,f['old_accepted'],f['newF_accepted'])))


def gates(pooled,cases):
    result=[]
    def add(name,actual,limit,passed,**context):result.append(dict(name=name,actual=actual,limit=limit,passed=bool(passed),**context))
    C,A=pooled['methods']['C']['singleton'],pooled['methods']['A']['singleton']
    add('pooled_primary_errors_no_more_than_A',C['errors'],A['errors'],C['errors']<=A['errors'])
    for case in CASES:
        c,a=cases[case]['methods']['C']['singleton'],cases[case]['methods']['A']['singleton']
        if c['denominator']!=a['denominator']:raise ValueError('Perphase gate denominator mismatch')
        add(case+'_primary_errors_no_more_than_A',c['errors'],a['errors'],c['errors']<=a['errors'])
    for domain in ('old_ambiguous_motion','all_primary'):
        v=pooled['acceptance'][domain]
        add(domain+'_acceptance_coverage_no_less_than_old_A',v['new_fused_fraction'],v['old_A_fraction'],
            v['denominator']>0 and v['new_fused_accepted']>=v['old_A_accepted'],
            denominator=v['denominator'],new_accepted=v['new_fused_accepted'],old_accepted=v['old_A_accepted'],
            qualification='These two acceptanceconstraints share a numerator here and are not independent evidence.')
    point=pooled['points'];Ccov,Ncov=point['methods']['C']['coverage'],point['methods']['N']['coverage']
    fixed=point['fixed_primary_C_vs_N']
    add('fullprimary_C_point_coverage_no_less_than_N',Ccov,Ncov,
        Ccov is not None and Ncov is not None and Ccov>=Ncov and fixed['C_complete_on_native_reference'],
        C_missing_on_native_reference=fixed['C_missing_on_native_reference'])
    for metric in ('rms','p95','maximum'):
        c,n=fixed['C_mm'][metric],fixed['N_mm'][metric]
        add('fixedprimary_point_'+metric+'_no_worse_than_N_plus1um',c,None if n is None else n+.001,
            c is not None and n is not None and c<=n+.001,units='mm',tolerance_m=1e-6,
            reference_count=fixed['reference_count'])
    return result


def event_row(case,i,frames):
    f=frames[case]
    return dict(case=case,row=int(i),time_s=float(f['time_s'][i]),truth_family=int(f['truth_family'][i]),
        truth_support_bits=int(f['truth_support_bits'][i]),old_accepted=bool(f['old_accepted'][i]),
        newF_accepted=bool(f['newF_accepted'][i]),newVG_accepted=bool(f['newVG_accepted'][i]),
        methods={m:dict(family=int(f[m+'__family'][i]),correct=bool(f[m+'__correct_family'][i]),
            point_m=f[m+'__point_m'][i],point_distance_mm=float(f[m+'__point_distance_mm'][i])) for m in METHODS})


def events(frames,protocol):
    anchors=protocol.get('fixed_events',[dict(case=c,time_s=t) for c,t in DEFAULT_EVENTS])
    fixed=[];harmful=[]
    for anchor in anchors:
        c=anchor['case'];indices=np.flatnonzero(abs(frames[c]['time_s']-float(anchor['time_s']))<=1e-9)
        if len(indices)!=1:raise ValueError('Fixed event clock mismatch')
        fixed.append(dict(anchor=anchor,result=event_row(c,int(indices[0]),frames)))
    for c in CASES:
        f=frames[c];mask=f['primary_dominant']&f['N__correct_family']&~f['A__correct_family']
        harmful += [event_row(c,int(i),frames) for i in np.flatnonzero(mask)]
    return dict(fixed_anchors=fixed,all_old_harmful=harmful)


def run(protocol_path,packet_root,output):
    if ref.sha(REFERENCE)!=REFERENCE_SHA:raise ValueError('Frozen metric reference changed')
    protocol=json.loads(protocol_path.read_text())
    if protocol['iteration']!='I-72':raise ValueError('Wrong experiment protocol')
    paths=[Path(__file__),REFERENCE,protocol_path]+[packet_root/c/'evaluation_packet.npz' for c in CASES]
    bindings=[dict(path=str(p),sha256=ref.sha(p)) for p in paths]
    output.mkdir(parents=True,exist_ok=False)
    ref.write(output/'registered_before_truth_read.json',dict(iteration='I-72',utc=datetime.now(timezone.utc).isoformat(),
        files=bindings,primary_policy='NewFUSEDaccepted physicalpoint/family elseN originalequivalentpoint/family; no winner selection.'))
    shutil.copy2(Path(__file__),output/'evaluate_selector.py');shutil.copy2(REFERENCE,output/'frozen_i70_metrics.py')
    shutil.copy2(protocol_path,output/'protocol.json')
    frames,cases={},{}
    for c in CASES:
        packet=ref.load(packet_root/c/'evaluation_packet.npz')
        if len(packet['time_s'])!=8801:raise ValueError('Fullregistered44s episode required')
        frames[c],cases[c]=evaluate_case(packet)
        if cases[c]['counts']['primary_window']!=8000:raise ValueError('Registeredhalf-open primarywindow changed')
        np.savez_compressed(output/(c+'_frames.npz'),**frames[c])
    pooled=aggregate(frames,cases);checks=gates(pooled,cases)
    if (pooled['counts']['primary_dominant']!=23423 or pooled['counts']['loaded']!=23894 or
        [pooled['methods'][m]['singleton']['errors'] for m in ('N','oldF','A')]!=[412,109,124]):
        raise ValueError('Frozenhistorical domains/baselinecounts not reproduced')
    if [pooled['cohorts'][k]['denominator'] for k in ('old_useful','old_harmful','old_accepted_wrong')]!=[294,6,18]:
        raise ValueError('Frozenhistorical correction cohorts changed')
    science=dict(iteration='I-72',new_physics_steps=0,live_controller_changed=False,parameters_fitted=False,
        primary_policy='C=newFUSEDaccepted candidate elseN family/equivalentpoint; allotherssecondary.',
        metric_reference_sha256=REFERENCE_SHA,cases=cases,pooled=pooled,gates=checks,
        all_gates_passed=all(g['passed'] for g in checks),failed_gates=[g['name'] for g in checks if not g['passed']],
        events=events(frames,protocol),
        interpretation='Retrospective dominant-support selector diagnostic. No multicontactclaim, no controller/trajectory improvement, no scorecalibration or confidenceprobability claim.',
        truth_scope='Last1ms sampled positive-pressurefamily loads/centroids every5ms; nativeNpoint isobserverequivalent, notNalt or truth.')
    for item in bindings:
        if ref.sha(item['path'])!=item['sha256']:raise ValueError('Frozeninput changed duringevaluation')
    ref.write(output/'science.json',science)
    ref.write(output/'artifact_manifest.json',dict(files=[dict(path=p.name,sha256=ref.sha(p),bytes=p.stat().st_size)
        for p in sorted(output.iterdir()) if p.is_file() and p.name!='artifact_manifest.json']))
    print(json.dumps(ref.safe(dict(failed_gates=science['failed_gates'],pooled=pooled)),allow_nan=False))


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    for k in ('protocol','packet-root','output'):p.add_argument('--'+k,type=Path,required=True)
    a=p.parse_args();run(a.protocol.resolve(),a.packet_root.resolve(),a.output.resolve())
