"""Synthetic storage contracts only; neither a solver nor a simulator runs."""
import unittest
import numpy as np
import packet_archive as archive


class Storage(unittest.TestCase):
    def test_active_packet_values_exact_with_padding_omitted(self):
        p=dict(kind=0,id=1,control_time_s=.005,substep=1,n=3,stride=4,nub=0,early=True,success=True,solver='Dantzig')
        for when in ('before','after'):
            a=np.arange(12,dtype='<f8').reshape(3,4);a[:,3]=np.nan
            p[when+'_A']=a.ravel()
            for key in ('x','b','lo','hi'):p[when+'_'+key]=np.array([-0.,1.,2.],dtype='<f8')
            p[when+'_findex']=np.array([-1,0,0],dtype='<i4')
        p['original_matrix']=p['before_A'].reshape(3,4)[:,:3].copy()
        restored=archive.decode(archive.canonical_bytes([p]))
        archive.verify_packets(restored,archive.packet_bindings([p]))
        np.testing.assert_array_equal(restored[0]['before_A'].reshape(3,4)[:,3],0.)

    def test_sparse_context_preserves_indices_and_marks_unknown(self):
        n=35;source=dict(control_end_index=np.repeat(np.arange(1,8),5),
            control_start_time_s=np.repeat(np.arange(7)*.005,5),control_end_time_s=np.repeat(np.arange(1,8)*.005,5),
            substep_index=np.tile(np.arange(1,6),7),contact_offsets=np.arange(n+1,dtype=np.int64),
            contact_rows=np.arange(n*16,dtype=float).reshape(n,16))
        roi=archive.select_trace(source,[1,3,5,7]);view=archive.matching_view(roi)
        self.assertEqual(len(roi['source_step_index']),20)
        np.testing.assert_array_equal(view['contact_rows'][roi['source_contact_index']],roi['contact_rows'])
        self.assertTrue(np.isnan(view['control_start_time_s'][5:10]).all())
        self.assertTrue(np.isnan(view['contact_rows'][5:10]).all())
        for i in roi['source_step_index']:
            np.testing.assert_array_equal(view['contact_offsets'][i:i+2],source['contact_offsets'][i:i+2])


if __name__=='__main__':unittest.main(verbosity=2)
