"""Post-primary F/V and stable-window description; no method/threshold changes."""
from pathlib import Path
import argparse
import json
import numpy as np


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--evaluation-root',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    if args.output.exists():raise FileExistsError(args.output)
    source=json.loads((args.evaluation_root/'science.json').read_text())
    result={'iteration':'I-60','scope':'Post-primary descriptive comparison; primary gate decisions unchanged','cases':{},'new_physics_runs':0}
    for case,original in source['cases'].items():
        with np.load(args.evaluation_root/(case+'_evaluation.npz'),allow_pickle=False)as z:
            data={key:z[key].copy()for key in z.files}
        masks={'all_recorded':np.ones(len(data['time_s']),bool),
               'primary_dominant':data['primary_mask']&data['truth_eligible'],
               'stable_primary_dominant':data['primary_mask']&data['truth_eligible']&data['stable_truth_mask']}
        case_report={}
        for name,mask in masks.items():
            first,second=data['fused__chosen_family'],data['velocity__chosen_family']
            faccept,vaccept=data['fused__accepted_family'],data['velocity__accepted_family']
            only_f=mask&(faccept>=0)&(vaccept<0)
            only_v=mask&(vaccept>=0)&(faccept<0)
            case_report[name]={'rows':int(mask.sum()),
                'raw_labels_different':int(np.sum(mask&(first!=second))),
                'accepted_labels_different':int(np.sum(mask&(faccept!=vaccept))),
                'fused_only_accepted_count':int(only_f.sum()),'velocity_only_accepted_count':int(only_v.sum()),
                'fused_only_accepted_eligible_wrong':int(np.sum(only_f&data['truth_eligible']&(faccept!=data['truth_family']))),
                'velocity_only_accepted_eligible_wrong':int(np.sum(only_v&data['truth_eligible']&(vaccept!=data['truth_family']))),
                'fused_only_accepted_times_s':data['time_s'][only_f].tolist(),
                'velocity_only_accepted_times_s':data['time_s'][only_v].tolist()}
        stable=original['windows']['stable_primary']
        primary=original['windows']['primary']
        case_report['primary_and_stable']={name:{'eligible':item['counts']['dominant_eligible'],
            'native_errors':item['native']['error_count'],
            'fused_errors':item['modes']['fused']['raw']['error_count'],
            'velocity_errors':item['modes']['velocity']['raw']['error_count'],
            'native_false_switches':item['native']['false_switch_count'],
            'fused_false_switches':item['modes']['fused']['raw']['false_switch_count']}
            for name,item in [('primary',primary),('stable_primary',stable)]}
        result['cases'][case]=case_report
    result['interpretation']='Identical labels at some/all recorded states do not prove general equivalence. Stable-truth agreement narrows a clock concern but cannot eliminate observer/filter/physics-clock effects or demonstrate causal contact accuracy.'
    args.output.write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps({case:{'primary_and_stable':value['primary_and_stable'],
                           'F_V_all_raw_difference':value['all_recorded']['raw_labels_different'],
                           'F_V_primary_accepted_difference':value['primary_dominant']['accepted_labels_different']}
                      for case,value in result['cases'].items()}))


if __name__=='__main__':main()
