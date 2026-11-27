"""Decode I69 private solver-call packets; preserve original array order."""
import argparse
import json
from pathlib import Path
import struct
import numpy as np


def read_packets(path):
    raw=Path(path).read_bytes();cursor=0;packets=[]
    def scalar(fmt):
        nonlocal cursor
        n=struct.calcsize(fmt)
        if cursor+n>len(raw):raise ValueError('Truncated scalar packet')
        out=struct.unpack_from(fmt,raw,cursor);cursor+=n
        return out[0] if len(out)==1 else out
    def array(dtype,n):
        nonlocal cursor
        count=np.dtype(dtype).itemsize*n
        if cursor+count>len(raw):raise ValueError('Truncated array packet')
        out=np.frombuffer(raw,dtype=dtype,count=n,offset=cursor).copy();cursor+=count
        return out
    while cursor<len(raw):
        magic,version,kind,ident,time,step=scalar('<IIIQdi')
        if magic!=0x49363950 or version!=1:raise ValueError('Unknown packet format')
        p=dict(kind=kind,id=ident,control_time_s=time,substep=step)
        if kind in (0,1):
            n,stride,nub,early,success=scalar('<5i')
            if not 1<=n<=4096 or stride!=(n if n<=1 else ((n+3)&~3)):raise ValueError('Bad solver dimensions')
            p.update(n=n,stride=stride,nub=nub,early=bool(early),success=bool(success),solver='Dantzig' if kind==0 else 'PGS')
            for when in ('before','after'):
                for key in ('A','x','b','lo','hi'):
                    p[when+'_'+key]=array('<f8',n*stride if key=='A' else n)
                p[when+'_findex']=array('<i4',n)
            p['original_matrix']=p['before_A'].reshape(n,stride)[:,:n].copy()
            if not np.isfinite(p['original_matrix']).all():raise ValueError('Nonfinite active matrix; padding is excluded')
        elif kind==2:
            parent,dim,offset,buffer,dt,mu1,mu2=scalar('<Qiiiddd')
            if dim not in (1,3):raise ValueError('Invalid contact dimension')
            p.update(parent_solver_id=parent,dim=dim,offset=offset,buffer=buffer,dt_s=dt,mu1=mu1,mu2=mu2,basis_available=(dim==3))
            for key,n in [('lambda',dim),('point',3),('normal',3),('force',3),('depth',1),('basis',6),('motion',3)]:
                p[key]=array('<f8',n)
        else:raise ValueError('Unknown packet kind')
        packets.append(p)
    return packets


def main():
    parser=argparse.ArgumentParser();parser.add_argument('path',type=Path);args=parser.parse_args()
    packets=read_packets(args.path)
    print(json.dumps(dict(packets=len(packets),solver_calls=sum(p['kind']<2 for p in packets),
        contacts=sum(p['kind']==2 for p in packets),n_values=sorted({p['n'] for p in packets if p['kind']<2}))))


if __name__=='__main__':main()
