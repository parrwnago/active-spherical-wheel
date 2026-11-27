"""Synthetic physical-point, fallback and boundary contracts for I72 consumer."""
import unittest
import numpy as np
from run_case import infer_case


def case_data():
    n=3;vector=np.array([[0.,0.,0.],[.01,0.,0.],[.04,0.,0.],[.05,0.,0.]])
    features=dict(family_index=np.array([0,0,1,1]),valid=np.ones((n,4),bool),frame_valid=np.ones(n,bool),
        excitation_m_s=np.array([.2,.001,.2]),normal_angle_rad=np.zeros((n,4)),
        velocity_residual_m_s=np.stack([vector]*n),velocity_residual_norm_m_s=np.stack([np.linalg.norm(vector,axis=-1)]*n),
        moment_residual_norm=np.zeros((n,4)))
    time=np.arange(n)*.005;anchor=np.zeros((n,3));anchor[2]=np.nan
    return dict(time_s=time,projected_features=features,
        world_points=np.tile(np.array([[.01,0,0],[.04,0,0],[.02,0,0],[.05,0,0]]),(n,1,1)),
        native_anchor_point_m=anchor,candidate_ids=np.array([20,7,60,9]),
        native_fallback=dict(time_s=time.copy(),native_family=np.ones(n,np.int64),
            native_point_m=np.tile([.123,.456,.789],(n,1)),native_support_loads_N=np.tile([0.,10.,0.,0.],(n,1))))


def all_bytes(data):
    result={}
    for key,value in data.items():
        if isinstance(value,dict):
            for name,array in value.items():result[key+'.'+name]=array.tobytes()
        else:result[key]=value.tobytes()
    return result


class ConsumerContracts(unittest.TestCase):
    def test_primary_uses_only_accepted_fused_else_exact_native_equivalent(self):
        data=case_data();out=infer_case(data)
        np.testing.assert_array_equal(out['fused__chosen_family'],[0,0,-1])
        np.testing.assert_array_equal(out['fused__accepted'],[True,False,False])
        np.testing.assert_array_equal(out['C_family'],[0,1,1])
        np.testing.assert_array_equal(out['C_point_m'][0],data['world_points'][0,0])
        np.testing.assert_array_equal(out['C_point_m'][1:],data['native_fallback']['native_point_m'][1:])
        np.testing.assert_array_equal(out['C_VG_family'],[0,1,1])

    def test_raw_point_is_not_replaced_by_native_when_abstaining(self):
        data=case_data();out=infer_case(data)
        np.testing.assert_array_equal(out['fused__point_m'][1],data['world_points'][1,0])
        self.assertTrue(np.isnan(out['fused__point_m'][2]).all())
        self.assertEqual(out['fused__chosen_candidate_id'][0],20)
        self.assertEqual(out['fused__chosen_row_index'][0],0)

    def test_changing_fallback_identity_and_point_cannot_change_ranking(self):
        data=case_data();old=infer_case(data)
        data['native_fallback']['native_family'][:]=3
        data['native_fallback']['native_point_m'][:]=[-7.,8.,9.]
        new=infer_case(data)
        for key in old:
            if '__' in key or key in ('selected_candidate_id','selected_mask','family_distance_squared_m2'):
                np.testing.assert_array_equal(old[key],new[key])
        np.testing.assert_array_equal(new['C_family'],[0,3,3])

    def test_raw_chosen_family_keeps_its_original_physical_point(self):
        data=case_data();data['projected_features']['velocity_residual_m_s'][:]=[[.04,0,0],[.05,0,0],[0,0,0],[.01,0,0]]
        data['projected_features']['velocity_residual_norm_m_s']=np.linalg.norm(data['projected_features']['velocity_residual_m_s'],axis=-1)
        out=infer_case(data)
        self.assertEqual(out['fused__chosen_family'][0],1)
        self.assertEqual(out['fused__chosen_candidate_id'][0],60)
        np.testing.assert_array_equal(out['fused__point_m'][0],data['world_points'][0,2])

    def test_readonly_inputs_are_not_mutated(self):
        data=case_data();before=all_bytes(data)
        for value in data.values():
            if isinstance(value,dict):
                for array in value.values():array.setflags(write=False)
            else:value.setflags(write=False)
        infer_case(data)
        self.assertEqual(before,all_bytes(data))

    def test_fallback_clock_and_family_corruption_are_rejected(self):
        data=case_data();data['native_fallback']['time_s'][1]+=.001
        with self.assertRaises(ValueError):infer_case(data)
        data=case_data();data['native_fallback']['native_family'][1]=5
        with self.assertRaises(ValueError):infer_case(data)


if __name__=='__main__':unittest.main(verbosity=2)
