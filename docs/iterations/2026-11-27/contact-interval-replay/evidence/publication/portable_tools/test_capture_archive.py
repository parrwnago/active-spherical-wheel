"""Small storage contracts, synthetic arrays only; no simulation."""
import unittest
import numpy as np
import capture_archive as archive
from frozen_substep_trace import bridge_state_to_qv


def fixture():
    raw = np.zeros((3, 24));raw[:, 5] = 1.;raw[:, 0] = np.arange(3)*.001
    raw[:, 2] = [.1, .2, .3];raw[:, 21] = .001
    rows = np.zeros((3, 16));rows[:, 0] = [0, 2, 6];rows[:, 5] = 1.
    trace = dict(bridge_state_before=raw[:2].copy(), bridge_state=raw[1:].copy(),
        contact_rows=rows.copy(), contact_offsets=np.array([0, 1, 3], np.int64),
        contact_rows_before=np.concatenate((rows[2:], rows[:1])),
        contact_offsets_before=np.array([0, 1, 2], np.int64),
        labels=np.array(['kept', 'unaltered']), values=np.array([-0., np.nan]))
    trace['qpos_before'], trace['qvel_before'] = bridge_state_to_qv(trace['bridge_state_before'])
    trace['qpos_after'], trace['qvel_after'] = bridge_state_to_qv(trace['bridge_state'])
    return trace


class StorageContract(unittest.TestCase):
    def test_continuity_roundtrip_keeps_dtype_shapes_bytes(self):
        original=fixture();encoded=archive.encode(original)
        self.assertEqual(len(encoded[archive.PREFIX+'state_exception_index']), 0)
        self.assertEqual(len(encoded[archive.PREFIX+'contact_exception_index']), 0)
        restored=archive.decode(encoded, bridge_state_to_qv)
        archive.validate(restored, {key:archive.binding(value) for key,value in original.items()})

    def test_state_and_ragged_contact_discontinuities_are_retained(self):
        original=fixture();original['bridge_state_before'][1, 2] += .125
        original['contact_rows_before']=np.vstack((original['contact_rows_before'][:1], np.ones((2,16))))
        original['contact_offsets_before']=np.array([0,1,3],np.int64)
        original['qpos_before'],original['qvel_before']=bridge_state_to_qv(original['bridge_state_before'])
        encoded=archive.encode(original)
        np.testing.assert_array_equal(encoded[archive.PREFIX+'state_exception_index'], [1])
        np.testing.assert_array_equal(encoded[archive.PREFIX+'contact_exception_index'], [1])
        restored=archive.decode(encoded,bridge_state_to_qv)
        archive.validate(restored,{key:archive.binding(value) for key,value in original.items()})

    def test_bit_change_is_rejected_including_signed_zero(self):
        original=fixture();expected={key:archive.binding(value) for key,value in original.items()}
        original['values'][0]=0.
        with self.assertRaises(AssertionError):archive.validate(original,expected)


if __name__=='__main__':unittest.main(verbosity=2)
