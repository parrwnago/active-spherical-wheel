"""I69 partial-context cached inputs, preserving mathematical packet values.

Uninitialized solver matrix padding is omitted and reconstructed as zeros.
This is not a byte-exact reconstruction of the raw native capture file.
Trace storage contains only20 registered steps, with original global indices;
unobserved portions of the matching adapter are NaN/unknown, never measurements.
"""
from __future__ import annotations
import argparse
import hashlib
import json
from pathlib import Path
import struct
import tempfile
import numpy as np
from read_probe import read_packets


TRACE_FIELDS=('control_start_time_s','control_end_time_s','substep_index','control_end_index')


def sha(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def binding(a):
    a=np.asarray(a)
    return dict(dtype=a.dtype.str,shape=list(a.shape),sha256=hashlib.sha256(np.ascontiguousarray(a).tobytes()).hexdigest())


def read(path):
    with np.load(path,allow_pickle=False) as p:return {k:p[k] for k in p.files}


def canonical_bytes(packets):
    """Serialize unchanged records, except explicitly nonmathematical A padding."""
    out=[]
    for p in packets:
        out.append(struct.pack('<IIIQdi',0x49363950,1,p['kind'],p['id'],p['control_time_s'],p['substep']))
        if p['kind'] in (0,1):
            n,stride=p['n'],p['stride']
            out.append(struct.pack('<5i',n,stride,p['nub'],p['early'],p['success']))
            for when in ('before','after'):
                a=np.zeros((n,stride),dtype='<f8')
                a[:,:n]=p[when+'_A'].reshape(n,stride)[:,:n]
                out.append(a.tobytes())
                for key in ('x','b','lo','hi','findex'):
                    out.append(np.asarray(p[when+'_'+key],dtype='<i4' if key=='findex' else '<f8').tobytes())
        elif p['kind']==2:
            out.append(struct.pack('<Qiiiddd',p['parent_solver_id'],p['dim'],p['offset'],p['buffer'],p['dt_s'],p['mu1'],p['mu2']))
            for key in ('lambda','point','normal','force','depth','basis','motion'):
                out.append(np.asarray(p[key],dtype='<f8').tobytes())
        else:raise ValueError('Unknown packet kind')
    return b''.join(out)


def decode(raw):
    with tempfile.NamedTemporaryFile(suffix='.bin') as stream:
        stream.write(raw);stream.flush()
        return read_packets(stream.name)


def packet_bindings(packets):
    records=[]
    for p in packets:
        fields={}
        for key,value in p.items():
            if isinstance(value,np.ndarray):
                if key in ('before_A','after_A'):
                    fields[key]=dict(active=binding(value.reshape(p['n'],p['stride'])[:,:p['n']]),
                        original_padded=binding(value),padding_policy='Original padding is not retained or claimed; decoder uses zeros.')
                else:fields[key]=dict(array=binding(value))
            else:fields[key]=dict(scalar=value)
        records.append(fields)
    return records


def verify_packets(packets,expected):
    if len(packets)!=len(expected):raise AssertionError('Packet count changed')
    array_count=0;scalar_count=0
    for p,fields in zip(packets,expected):
        if set(p)!=set(fields):raise AssertionError('Packet field set changed')
        for key,spec in fields.items():
            value=p[key]
            if 'active' in spec:
                a=value.reshape(p['n'],p['stride'])
                if binding(a[:,:p['n']])!=spec['active'] or np.any(a[:,p['n']:]!=0):raise AssertionError('Active matrix changed or padding not zero')
                array_count+=1
            elif 'array' in spec:
                if binding(value)!=spec['array']:raise AssertionError('Packet array changed: '+key)
                array_count+=1
            else:
                if type(value)!=type(spec['scalar']) or value!=spec['scalar']:raise AssertionError('Packet scalar changed: '+key)
                scalar_count+=1
    return dict(packet_count=len(packets),all_active_matrices_and_other_arrays_byte_exact=True,
                array_fields=array_count,scalar_fields=scalar_count,padding='Omitted; decoder supplies zero nonmathematical padding.')


def select_trace(trace,target_indices):
    index=np.flatnonzero(np.isin(trace['control_end_index'],target_indices))
    if len(index)!=5*len(target_indices):raise AssertionError('Expected exactly20 fixed steps')
    for target in target_indices:
        part=index[trace['control_end_index'][index]==target]
        if not np.array_equal(trace['substep_index'][part],np.arange(1,6)):raise AssertionError('Missing fixed substeps')
    starts=trace['contact_offsets'][index];ends=trace['contact_offsets'][index+1]
    contacts=np.concatenate([np.arange(a,b,dtype=np.int64) for a,b in zip(starts,ends)])
    out={key:trace[key][index] for key in TRACE_FIELDS}
    out.update(source_step_index=index,source_contact_index=contacts,
        contact_start_index=starts,contact_end_index=ends,contact_rows=trace['contact_rows'][contacts],
        original_step_count=np.asarray(len(trace['control_start_time_s']),np.int64),
        original_contact_count=np.asarray(len(trace['contact_rows']),np.int64))
    return out


def matching_view(roi):
    n=int(roi['original_step_count']);m=int(roi['original_contact_count'])
    out={}
    for key in TRACE_FIELDS:
        a=roi[key];fill=np.nan if np.issubdtype(a.dtype,np.floating) else -1
        out[key]=np.full(n,fill,dtype=a.dtype);out[key][roi['source_step_index']]=a
    out['contact_rows']=np.full((m,16),np.nan,dtype=roi['contact_rows'].dtype)
    out['contact_rows'][roi['source_contact_index']]=roi['contact_rows']
    out['contact_offsets']=np.full(n+1,-1,dtype=roi['contact_start_index'].dtype)
    out['contact_offsets'][roi['source_step_index']]=roi['contact_start_index']
    out['contact_offsets'][roi['source_step_index']+1]=roi['contact_end_index']
    out['context_observed']=np.zeros(n,bool);out['context_observed'][roi['source_step_index']]=True
    out['contact_observed']=np.zeros(m,bool);out['contact_observed'][roi['source_contact_index']]=True
    return out


def metadata(package):
    package=Path(package);data=json.loads((package/'packet_manifest.json').read_text())
    for row in data['portable_files']:
        if sha(package/row['path'])!=row['sha256']:raise AssertionError('Portable file changed: '+row['path'])
    return data


def load_packets(package):
    package=Path(package);data=metadata(package)
    raw=read(package/'mathematical_packets.npz')['canonical_bytes'].tobytes()
    packets=decode(raw);verify_packets(packets,data['packet_bindings'])
    return packets


def load_trace(package,label='new'):
    package=Path(package);data=metadata(package);entry=data['traces'][label]
    roi=read(package/entry['payload'])
    if {k:binding(v) for k,v in roi.items()}!=entry['selected_array_bindings']:raise AssertionError('Selected trace values changed')
    return matching_view(roi)


def pack(args):
    destination=Path(args.output)
    if destination.exists():raise FileExistsError(destination)
    seal=json.loads(Path(args.seal).read_text())
    for row in seal['files']:
        if sha(row['path'])!=row['sha256']:raise AssertionError('Prepacking input/source changed: '+row['path'])
    protocol=json.loads(Path(args.protocol).read_text());targets=[x['control_end_index'] for x in protocol['targets']]
    packets=read_packets(args.capture);bindings=packet_bindings(packets)
    encoded=canonical_bytes(packets);verification=verify_packets(decode(encoded),bindings)
    destination.mkdir(parents=True)
    np.savez_compressed(destination/'mathematical_packets.npz',canonical_bytes=np.frombuffer(encoded,dtype=np.uint8))
    entries={};payloads={}
    for label,path in (('new',args.trace),('historical',args.historical_trace)):
        original=read(path);roi=select_trace(original,targets);fields={k:binding(v) for k,v in roi.items()}
        duplicate=next((name for name,old in payloads.items() if old==fields),None)
        name=duplicate or label+'_matching_contexts.npz'
        if duplicate is None:np.savez_compressed(destination/name,**roi);payloads[name]=fields
        view=matching_view(roi)
        for i in roi['source_step_index']:
            lo,hi=original['contact_offsets'][i:i+2]
            if binding(original['contact_rows'][lo:hi])!=binding(view['contact_rows'][lo:hi]):raise AssertionError('Selected original contact values changed')
        entries[label]=dict(original_file=str(path),original_sha256=sha(path),payload=name,
            selected_steps=len(roi['source_step_index']),selected_contact_rows=len(roi['source_contact_index']),
            selected_array_bindings=fields,
            scope='Partial20-step context only. Other contexts/rows are unknown NaN or integer−1; no full trace restoration claim.')
    data=dict(schema_version=1,numpy_version=np.__version__,original_capture_file=str(args.capture),
        original_capture_sha256=sha(args.capture),original_reader_sha256=sha(Path(__file__).parent/'read_probe.py'),
        original_capture_scope='Raw original file remains local. Padded A columns deliberately omitted; portable record is not raw-file-byte restoration.',
        packet_bindings=bindings,traces=entries,target_control_end_indices=targets,portable_files=[])
    files=sorted(destination.glob('*.npz'))+sorted((destination.parent/'portable_tools').glob('*.py'))
    import os
    for path in files:data['portable_files'].append(dict(path=os.path.relpath(path,destination),bytes=path.stat().st_size,sha256=sha(path)))
    (destination/'packet_manifest.json').write_text(json.dumps(data,indent=2,sort_keys=True,allow_nan=False)+'\n')
    verify_packets(load_packets(destination),bindings)
    for label in entries:load_trace(destination,label)
    verification.update(compact_npz_bytes=sum(p.stat().st_size for p in destination.glob('*.npz')),
        traces={k:dict(selected_steps=v['selected_steps'],selected_contact_rows=v['selected_contact_rows']) for k,v in entries.items()})
    (destination/'packing_verification.json').write_text(json.dumps(verification,indent=2,sort_keys=True)+'\n')
    print(json.dumps(verification,sort_keys=True))


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--capture',type=Path,required=True)
    p.add_argument('--trace',type=Path,required=True);p.add_argument('--historical-trace',type=Path,required=True)
    p.add_argument('--protocol',type=Path,required=True);p.add_argument('--seal',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True);pack(p.parse_args())


if __name__=='__main__':main()
