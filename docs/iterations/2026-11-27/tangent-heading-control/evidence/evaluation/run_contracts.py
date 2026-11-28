"""Seal and run focused synthetic contracts; never opens new case outputs."""
from pathlib import Path
from datetime import datetime,timezone
import argparse,hashlib,json,shutil,subprocess,sys

BASE=Path(__file__).resolve().parents[1]
ROOT=BASE.parents[1]


def sha(path):return hashlib.sha256(path.read_bytes()).hexdigest()
def save(path,value):path.write_text(json.dumps(value,indent=2,sort_keys=True)+'\n')


def main():
    p=argparse.ArgumentParser();p.add_argument('--stage',required=True);p.add_argument('--omit-unregistered-protocol',action='store_true');p.add_argument('--worker',action='store_true');args=p.parse_args()
    if args.worker:
        import unittest,test_metrics
        suite=unittest.defaultTestLoader.loadTestsFromModule(test_metrics)
        def flatten(s):
            for test in s:
                if isinstance(test,unittest.TestSuite):yield from flatten(test)
                else:yield test
        tests=[t for t in flatten(suite) if not(args.omit_unregistered_protocol and t._testMethodName=='test_actual_I75_protocol_and_initialcondition_change')]
        outcome=unittest.TextTestRunner(verbosity=2).run(unittest.TestSuite(tests));sys.exit(0 if outcome.wasSuccessful() else 1)
    stage=BASE/'evaluation'/args.stage;stage.mkdir(exist_ok=False)
    paths=[Path(__file__),BASE/'evaluation/metrics.py',BASE/'evaluation/evaluate.py',BASE/'evaluation/test_metrics.py',BASE/'evaluation/protocol_metrics_fragment.json',
           BASE/'adapter/tangent_heading_adapter.py',BASE/'campaign_tools/run_case.py',BASE/'runtime/route.py',BASE/'runtime/heading_reference.py']
    if not args.omit_unregistered_protocol:paths.append(BASE/'protocol/protocol.json')
    bindings={str(path.relative_to(ROOT)):dict(sha256=sha(path),bytes=path.stat().st_size) for path in paths}
    for source in paths:
        target=stage/'source'/source.relative_to(ROOT);target.parent.mkdir(parents=True,exist_ok=True);shutil.copy2(source,target)
    save(stage/'registered_before_tests.json',dict(utc=datetime.now(timezone.utc).isoformat(),files=bindings,
        no_physics=True,no_new_case_arrays=True,protocol_contract_omitted_until_registration=args.omit_unregistered_protocol))
    command=[sys.executable,str(Path(__file__).resolve()),'--stage',args.stage,'--worker']
    if args.omit_unregistered_protocol:command.append('--omit-unregistered-protocol')
    with (stage/'unittest.log').open('w') as handle:completed=subprocess.run(command,stdout=handle,stderr=subprocess.STDOUT)
    unchanged=all(sha(ROOT/name)==value['sha256'] for name,value in bindings.items())
    save(stage/'receipt.json',dict(utc=datetime.now(timezone.utc).isoformat(),command=command,exit_code=completed.returncode,
         source_inputs_unchanged=unchanged,no_physics=True,no_new_case_arrays=True,full_repo_suite=False,
         protocol_contract_omitted_until_registration=args.omit_unregistered_protocol))
    print((stage/'unittest.log').read_text(),end='')
    if not unchanged:raise ValueError('Contract sources changed while testing')
    sys.exit(completed.returncode)


if __name__=='__main__':main()
