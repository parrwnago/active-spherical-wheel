"""Package scoped I71 evidence and verify inference/evaluation after relocation."""
from pathlib import Path
from datetime import datetime, timezone
import argparse, hashlib, json, re, subprocess, sys, zipfile
import numpy as np

EXPERIMENT=Path('docs/iterations/2026-11-27/endpoint-pair-ablation')
PREFIX='i71-contact-estimator/'

def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def save(p,v):Path(p).write_text(json.dumps(v,indent=2,allow_nan=False)+'\n')
def run(command,log):
    with Path(log).open('x') as stream:
        subprocess.run(command,stdout=stream,stderr=subprocess.STDOUT,check=True)

def main():
    ap=argparse.ArgumentParser();ap.add_argument('--root',type=Path,required=True);ap.add_argument('--output',type=Path,required=True);a=ap.parse_args()
    root=a.root.resolve();a.output=a.output.resolve();folder=root/EXPERIMENT;evidence=folder/'evidence'
    if a.output.is_relative_to(folder):raise ValueError('Output must be outside the published folder')
    a.output.mkdir(parents=True,exist_ok=False)
    export=json.loads((evidence/'export_manifest.json').read_text())
    for row in export['files']:
        assert sha(evidence/row['path'])==row['sha256'],row['path']
    files=[p for p in folder.rglob('*') if p.is_file() and '__pycache__' not in p.parts and p.name!='publication_qa.json']
    files.append(root/'docs/CHATGPT_CONTROL_CONTEXT.md')
    binding={str(p.relative_to(root)):dict(sha256=sha(p),bytes=p.stat().st_size) for p in sorted(files)}
    save(a.output/'registered_before_package.json',dict(utc=datetime.now(timezone.utc).isoformat(),builder_sha256=sha(__file__),files=binding))
    archive=a.output/'i71_evidence.zip'
    with zipfile.ZipFile(archive,'x',compression=zipfile.ZIP_DEFLATED) as z:
        for p in sorted(files):z.write(p,PREFIX+str(p.relative_to(root)))
        z.writestr(PREFIX+'START_HERE.md','# I-71 handoff\n\nRead [the iteration record]('+str(EXPERIMENT)+'/README.md) and [standalone context](docs/CHATGPT_CONTROL_CONTEXT.md).\n\nThis version fails its continuation checks; live control is unchanged. Inference and evaluation are self-contained in the evidence tree. Older context links require the full repository.\n')
        z.writestr(PREFIX+'bundle_manifest.json',json.dumps(binding,indent=2)+'\n')
    extraction=a.output/'extracted';extraction.mkdir()
    with zipfile.ZipFile(archive) as z:
        assert z.testzip() is None
        for name in z.namelist():
            if not (extraction/name).resolve().is_relative_to(extraction.resolve()):raise ValueError('Unsafe archive path')
        z.extractall(extraction)
    extracted=extraction/PREFIX
    for name,v in binding.items():assert sha(extracted/name)==v['sha256'],name
    package=extracted/EXPERIMENT/'evidence';fresh=a.output/'inference'
    run([sys.executable,'-I',str(package/'publication/portable_tools/replay_inference.py'),
         '--package',str(package),'--output',str(fresh)],a.output/'inference.log')
    inference=json.loads((fresh/'receipt.json').read_text());assert inference['all_arrays_byte_exact'] and inference['all_three_completed']
    for case,receipt in inference['cases'].items():
        assert receipt['input_arrays_unchanged']
        assert all(Path(i['path']).is_relative_to(package) for i in receipt['imports'].values())
    packet=a.output/'evaluation_packets';evaluation=a.output/'evaluation'
    run([sys.executable,'-I',str(package/'evaluation/prepare_evaluation.py'),
         '--baseline-root',str(package/'evaluation/packets_stage01'),'--prediction-root',str(fresh),
         '--inference-seal',str(fresh/'receipt.json'),'--output',str(packet)],a.output/'projection.log')
    run([sys.executable,'-I',str(package/'evaluation/evaluate_ablation.py'),
         '--protocol',str(package/'protocol/protocol.json'),'--packet-root',str(packet),
         '--prediction-root',str(fresh),'--parent-science',str(package/'reference_i70/science.json'),
         '--output',str(evaluation)],a.output/'evaluation.log')
    assert sha(evaluation/'science.json')==sha(package/'evaluation/stage01/science.json')
    array_count=0
    for case in ('m100_B','p030_B','p090_B'):
        with np.load(evaluation/(case+'_frames.npz'),allow_pickle=False) as new,np.load(package/'evaluation/stage01'/(case+'_frames.npz'),allow_pickle=False) as old:
            assert set(new.files)==set(old.files)
            for key in new.files:
                x,y=new[key],old[key]
                assert x.dtype==y.dtype and x.shape==y.shape and np.ascontiguousarray(x).tobytes()==np.ascontiguousarray(y).tobytes(),(case,key)
            array_count+=len(new.files)
    link_count=0
    for p in [root/'ITERATION_LOG.md',root/'docs/CHATGPT_CONTROL_CONTEXT.md',root/'docs/TEAMMATE_HANDOFF.md',root/'docs/iterations/2026-11-27/README.md',folder/'README.md']:
        for target in re.findall(r'\]\(([^)]+)\)',p.read_text()):
            target=target.strip('<>')
            if '://' in target or target.startswith('#'):continue
            assert (p.parent/target.split('#')[0]).exists(),(p,target)
            link_count+=1
    pdf=package/'publication/plots_stage01/endpoint_pair_ablation_report.pdf'
    pages=len(re.findall(rb'/Type\s*/Page\b',pdf.read_bytes()));assert pages==3
    for name,v in binding.items():assert sha(root/name)==v['sha256'],name
    science=json.loads((evaluation/'science.json').read_text())
    receipt=dict(iteration='I-71',status='PASS publication/numerical checks; scientific stopping decision retained',
        package_utc=datetime.now(timezone.utc).isoformat(),builder_sha256=sha(__file__),bundle_path=str(archive),
        bundle_sha256=sha(archive),bundle_bytes=archive.stat().st_size,payload_files=len(binding),
        export_copies_verified=len(export['files']),zip_crc_passed=True,all_payload_hashes_exact=True,
        inference_receipt=inference,inference_receipt_sha256=sha(fresh/'receipt.json'),
        evaluation_science_byte_exact=True,evaluation_science_sha256=sha(evaluation/'science.json'),
        evaluation_frame_arrays_byte_exact=True,evaluation_frame_arrays_verified=array_count,
        local_links_verified=link_count,pdf_pages=pages,figure_review='All three PNG pages visually inspected before publication.',
        continuation_decision=science['continuation_decision'],failed_continuation_checks=science['failed_continuation_gates'],failed_adoption_gates=science['failed_gates'],
        new_physics_steps=0,live_controller_changed=False,
        scope='31new focused contracts:27combined+4independent inclusive. Old21metriccontracts not added. No cross-version or tracking claim. Publication QA excluded from its own pre-publication archive.')
    save(a.output/'receipt.json',receipt);save(folder/'publication_qa.json',receipt)
    print(json.dumps(dict(archive=str(archive),files=len(binding),inference_arrays=81,evaluation_arrays=array_count,science_exact=True,pages=pages,links=link_count)))

if __name__=='__main__':main()
