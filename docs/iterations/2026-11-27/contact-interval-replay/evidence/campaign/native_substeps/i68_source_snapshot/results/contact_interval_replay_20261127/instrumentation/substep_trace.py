"""I68 harness-only 1 ms state/contact capture within unchanged 5 ms holds.

The only stepping change is original_sim.run(5) -> five original_sim.run(1)
calls in the registered window. No command, observer, mirror pull/forward or
hidden-state reset is added. Native parity must be checked by the caller.
"""
from contextlib import contextmanager
import hashlib
import json
from pathlib import Path
from unittest.mock import patch

import numpy as np


STATE_FIELDS = ('time', 'iterations', 'x', 'y', 'z', 'qw', 'qx', 'qy', 'qz',
                'vx', 'vy', 'vz', 'wx', 'wy', 'wz', 'q_shaft', 'q_A', 'q_B',
                'dq_shaft', 'dq_A', 'dq_B', 'step_s', 'resets', 'reset_pending')
CONTACT_FIELDS = ('tag', 'px', 'py', 'pz', 'nx', 'ny', 'nz', 'fx', 'fy', 'fz',
                  'depth', 'friction', 'admitted', 'reserved_0', 'reserved_1', 'reserved_2')
CONTACT_TAGS = ('shell_A_smooth', 'shell_B_smooth', 'belt_A_smooth', 'belt_B_smooth',
                'shell_A_rim', 'shell_B_rim', 'body')


def bridge_state_to_qv(state):
    """Offline conversion of cached raw bridge state to native qpos/qvel.

    Bridge base linear/angular velocities are world-frame; a MuJoCo free joint
    uses body-frame angular velocity. This derives coordinates from the same
    raw reading, not an additional state measurement or mirror pull.
    """
    state=np.asarray(state,float)
    if state.ndim<1 or state.shape[-1]!=24 or not np.isfinite(state).all():
        raise ValueError('Expected finite cached native bridge states24')
    norm=np.linalg.norm(state[...,5:9],axis=-1)
    if np.any(norm<=1e-12):raise ValueError('Bridge quaternion must be nonzero')
    quat=state[...,5:9]/norm[...,None];w,x,y,z=np.moveaxis(quat,-1,0)
    rotation=np.empty(state.shape[:-1]+(3,3))
    rotation[...,0,0]=1.-2.*(y*y+z*z);rotation[...,0,1]=2.*(x*y-w*z);rotation[...,0,2]=2.*(x*z+w*y)
    rotation[...,1,0]=2.*(x*y+w*z);rotation[...,1,1]=1.-2.*(x*x+z*z);rotation[...,1,2]=2.*(y*z-w*x)
    rotation[...,2,0]=2.*(x*z-w*y);rotation[...,2,1]=2.*(y*z+w*x);rotation[...,2,2]=1.-2.*(x*x+y*y)
    qpos=np.empty(state.shape[:-1]+(10,));qvel=np.empty(state.shape[:-1]+(9,))
    qpos[...,:3]=state[...,2:5];qpos[...,3:7]=quat;qpos[...,7:10]=state[...,15:18]
    qvel[...,:3]=state[...,9:12];qvel[...,3:6]=np.einsum('...ji,...j->...i',rotation,state[...,12:15]);qvel[...,6:9]=state[...,18:21]
    return qpos,qvel


class SubstepTrace:
    """One physical plant, one recording window, no controller intervention.

    Use ``with trace.instrument(): ...original native loop...`` and then
    ``trace.save(new_npz_path)``. Rows are samples AFTER a completed 1 ms solve;
    bridge_state_before records the preceding bridge state without integration.
    Both relative controller time and the plugin's separate clock are retained.
    All native contact rows, including body and rejected records, are copied.
    """

    def __init__(self, physical_plant, start_s=0., end_s=44.,
                 physics_dt_s=.001, clock_tolerance_s=1e-9):
        values = np.asarray([start_s, end_s, physics_dt_s, clock_tolerance_s], float)
        if not np.isfinite(values).all() or start_s < 0 or end_s <= start_s or physics_dt_s <= 0 or clock_tolerance_s < 0:
            raise ValueError('Finite ordered recording bounds and positive physics step required')
        self.plant, self.sim = physical_plant, physical_plant.sim
        self.start_s, self.end_s = float(start_s), float(end_s)
        self.physics_dt_s = float(physics_dt_s)
        self.clock_tolerance_s = float(clock_tolerance_s)
        self.control_dt_s = float(physical_plant.dt)
        self.substeps = int(physical_plant.substeps)
        if (self.substeps != 5 or not np.isfinite(self.control_dt_s) or
                abs(self.control_dt_s - self.substeps*self.physics_dt_s) > 1e-12):
            raise ValueError('This registered recorder requires five physics steps per control hold')
        self._samples = []
        self._active = None
        self._installed = False
        self.completed_holds = 0
        self.failed_holds = 0
        self.unsplit_run_calls = 0
        # Tags identify logical physical collision patches. Mirror IDs/names
        # are static known-model mappings, not Gazebo entity IDs.
        self.tag_mirror_geom_ids=np.full(7,-1,np.int64);self.tag_mirror_body_ids=np.full(7,-1,np.int64)
        self.tag_mirror_geom_names=np.full(7,'unknown',dtype='<U64');self.tag_mirror_body_names=np.full(7,'unknown',dtype='<U64')
        ids=getattr(physical_plant,'_geom_ids',None);model=getattr(physical_plant,'model',None)
        if ids is not None and model is not None:
            ids=np.asarray(ids,np.int64)
            if ids.shape!=(7,) or np.any(ids<0):raise ValueError('Expected all seven native tag-to-mirror geom IDs')
            self.tag_mirror_geom_ids=ids.copy();self.tag_mirror_body_ids=np.asarray(model.geom_bodyid[ids],np.int64).copy()
            for k,(geom,body) in enumerate(zip(ids,self.tag_mirror_body_ids)):
                self.tag_mirror_geom_names[k]=str(model.geom(int(geom)).name)
                self.tag_mirror_body_names[k]=str(model.body(int(body)).name)

    def _in_window(self, time_s):
        return (time_s >= self.start_s-self.clock_tolerance_s and
                time_s < self.end_s-self.clock_tolerance_s)

    def _state(self):
        state = np.asarray(self.sim.state(), dtype=np.float64).copy()
        if state.shape != (len(STATE_FIELDS),) or not np.isfinite(state).all():
            raise ValueError('Expected the finite native 24-field bridge state')
        return state

    def _contacts(self):
        rows = np.asarray(self.sim.contacts(), dtype=np.float64).copy()
        if rows.ndim != 2 or rows.shape[1] != len(CONTACT_FIELDS) or not np.isfinite(rows).all():
            raise ValueError('Expected all finite native bridge contact rows with stride16')
        if len(rows) and (not np.equal(rows[:, 0], np.round(rows[:, 0])).all() or
                          np.any((rows[:, 0] < 0) | (rows[:, 0] >= len(CONTACT_TAGS)))):
            raise ValueError('Unknown bridge contact tag; do not silently filter it')
        return rows

    def _held(self):
        command = np.asarray(self.plant.last_input, dtype=np.float64).copy()
        disturbance = np.asarray(self.plant.last_disturbance, dtype=np.float64).copy()
        friction = float(self.plant.ground['friction'] if self.plant.ground else self.plant.nominal_friction)
        if command.shape != (5,) or disturbance.shape != (6,) or not np.isfinite(np.r_[command, disturbance, friction]).all():
            raise ValueError('Finite actual held input5, world disturbance6 and friction required')
        return command, disturbance, friction

    @contextmanager
    def instrument(self):
        if self._installed:
            raise RuntimeError('Recorder cannot be nested or installed twice concurrently')
        original_step, original_run = self.plant.step, self.sim.run

        def run(iterations):
            active = self._active
            if active is None:
                self.unsplit_run_calls += 1
                return original_run(iterations)
            if iterations != self.substeps or active['run_calls'] != 0:
                raise AssertionError('Exactly one native run(5) is expected inside each physical hold')
            active['run_calls'] += 1
            held, disturbance, friction = self._held()  # Original step has already issued its command.
            before = self._state()
            before_contacts = self._contacts()
            result = None
            for k in range(self.substeps):
                result = original_run(1)
                after = self._state()
                contacts = self._contacts()
                if abs((after[0]-before[0])-self.physics_dt_s) > 1e-8:
                    raise AssertionError('Plugin clock did not advance by one registered physics step')
                if after[1]-before[1] != 1 or abs(after[21]-self.physics_dt_s) > 1e-12:
                    raise AssertionError('Plugin iteration/step size does not match one physical solve')
                if after[22] != before[22] or after[23] != 0:
                    raise AssertionError('A reset cannot occur inside a recorded held interval')
                current_held, current_disturbance, current_friction = self._held()
                if (not np.array_equal(held, current_held) or
                        not np.array_equal(disturbance, current_disturbance) or friction != current_friction):
                    raise AssertionError('Native held input changed inside its five physics steps')
                self._samples.append(dict(
                    time_s=active['start_s']+(k+1)*self.physics_dt_s,
                    control_start_time_s=active['start_s'],
                    control_end_time_s=active['start_s']+self.control_dt_s,
                    hold_index=int(round(active['start_s']/self.control_dt_s)),
                    substep_index=k+1, bridge_state_before=before.copy(), bridge_state=after.copy(),
                    contact_rows_before=before_contacts, contact_rows=contacts,
                    held_input=held.copy(), disturbance_world=disturbance.copy(),
                    friction=friction, hold_completed=False))
                before = after
                before_contacts = contacts
            return result

        def step(command, disturbance=None):
            now = float(self.plant.data.time)
            if not np.isfinite(now):
                raise ValueError('Finite relative control time required')
            if not self._in_window(now):
                return original_step(command, disturbance)
            if self._active is not None:
                raise RuntimeError('Physical hold instrumentation cannot overlap')
            first = len(self._samples)
            self._active = dict(start_s=now, run_calls=0)
            try:
                result = original_step(command, disturbance)
                if self._active['run_calls'] != 1 or len(self._samples)-first != self.substeps:
                    raise AssertionError('Completed physical hold must contain exactly five recorded solves')
                for sample in self._samples[first:]:
                    sample['hold_completed'] = True
                self.completed_holds += 1
                return result
            except BaseException:
                self.failed_holds += 1
                raise
            finally:
                self._active = None

        self._installed = True
        try:
            with patch.object(self.plant, 'step', step), patch.object(self.sim, 'run', run):
                yield self
        finally:
            self._installed = False

    def arrays(self):
        """Return independent fixed-dtype arrays; no object arrays or pickle."""
        if self._active is not None:
            raise RuntimeError('Finish the current physical hold before exporting')
        samples, count = self._samples, len(self._samples)
        offsets = np.r_[0, np.cumsum([len(row['contact_rows']) for row in samples], dtype=np.int64)]
        contacts = (np.concatenate([row['contact_rows'] for row in samples], axis=0)
                    if samples else np.empty((0, len(CONTACT_FIELDS)), np.float64))
        before_offsets=np.r_[0,np.cumsum([len(row['contact_rows_before']) for row in samples],dtype=np.int64)]
        before_contacts=(np.concatenate([row['contact_rows_before'] for row in samples],axis=0)
                         if samples else np.empty((0,len(CONTACT_FIELDS)),np.float64))
        result = dict(contact_rows=contacts, contact_offsets=offsets.astype(np.int64),
                      contact_rows_before=before_contacts,contact_offsets_before=before_offsets.astype(np.int64),
                      contact_tags=contacts[:, 0].astype(np.int64), contact_admitted=(contacts[:, 12] >= .5),
                      state_fields=np.asarray(STATE_FIELDS), contact_fields=np.asarray(CONTACT_FIELDS),
                      contact_tag_names=np.asarray(CONTACT_TAGS),
                      tag_mirror_geom_ids=self.tag_mirror_geom_ids.copy(),tag_mirror_body_ids=self.tag_mirror_body_ids.copy(),
                      tag_mirror_geom_names=self.tag_mirror_geom_names.copy(),tag_mirror_body_names=self.tag_mirror_body_names.copy())
        for key, shape in [('bridge_state_before', (len(STATE_FIELDS),)), ('bridge_state', (len(STATE_FIELDS),)),
                           ('held_input', (5,)), ('disturbance_world', (6,))]:
            result[key] = np.stack([row[key] for row in samples]) if count else np.empty((0, *shape), np.float64)
        for key in ('time_s', 'control_start_time_s', 'control_end_time_s', 'friction'):
            result[key] = np.asarray([row[key] for row in samples], dtype=np.float64)
        for key in ('hold_index', 'substep_index'):
            result[key] = np.asarray([row[key] for row in samples], dtype=np.int64)
        result['hold_completed'] = np.asarray([row['hold_completed'] for row in samples], dtype=bool)
        result['bridge_time_s'] = result['bridge_state'][:, 0].copy()
        result['bridge_time_before_s']=result['bridge_state_before'][:,0].copy()
        result['bridge_time_after_s']=result['bridge_state'][:,0].copy()
        result['step_dt_s']=result['bridge_state'][:,21].copy()
        result['control_end_index']=result['hold_index']+1
        result['bridge_iterations'] = result['bridge_state'][:, 1].astype(np.int64)
        result['qpos_before'],result['qvel_before']=bridge_state_to_qv(result['bridge_state_before'])
        result['qpos_after'],result['qvel_after']=bridge_state_to_qv(result['bridge_state'])
        result['contact_mirror_body_ids']=self.tag_mirror_body_ids[result['contact_tags']]
        return result

    def save(self, path):
        """Write a NEW NPZ; never overwrite an earlier trace, including failures."""
        path = Path(path)
        if path.exists() or path.is_symlink():
            raise FileExistsError(path)
        arrays = self.arrays()
        metadata = dict(schema_version=2, helper_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
                        window_start_s=self.start_s, window_end_s=self.end_s,
                        clock_tolerance_s=self.clock_tolerance_s, control_dt_s=self.control_dt_s,
                        physics_dt_s=self.physics_dt_s, substeps=self.substeps,
                        completed_holds=self.completed_holds, failed_holds=self.failed_holds,
                        unsplit_run_calls=self.unsplit_run_calls,
                        input_timing='Actual plant.last_input after original command issuance; same held command for all five solves.',
                        state_timing='Raw bridge state immediately before/after each completed1ms solve. Plugin clock is separate from relative control time.',
                        contact_timing='Full native last-solve contact rows16 after every1ms; force samples are not labeled exact solver impulse records.',
                        pre_contact_timing='Before rows are the preceding reported solve snapshot, copied once before the hold and carried from each post-step copy; not a continuous force measurement.',
                        force_convention='Columns7:10 are the published force on the wheel/body in world N, not an impulse to divide by dt again. Normal points ground-to-wheel; no normalization/filtering is applied here.',
                        couple_coverage='Current bridge does not export per-contact torque/couple or total solver constraint wrench. Reserved columns13:16 are zero-filled ABI padding, not measured couples.',
                        per_contact_couple_available=False,total_constraint_wrench_available=False,
                        mapping_convention='Logical bridge tag names plus known mirror geom/body mapping; mirror IDs are not Gazebo entity IDs.',
                        mirror_mapping_available=bool(np.all(self.tag_mirror_body_ids>=0)),
                        qvel_convention='Offline-derived from cached raw24: base linear velocity world, free-joint angular velocity body, joint rates scalar; no extra pull/forward.',
                        dt_convention='step_dt_s is the bridge reported actual physics step. Native-relative after-step time_s and separate raw before/after bridge clocks are retained.',
                        disturbance_convention='Applied world input wrench is passed to fork AddWorldWrench at configured fork COM; it is not a contact-couple measurement.',
                        stepping_scope='Only registered physical.step holds splitrun5 into five run1. No extra command/observer/pull/forward/reset.',
                        parity_status='Caller must verify unchanged prefix/control and physical evolution before causal interpretation.')
        arrays['metadata_json'] = np.asarray(json.dumps(metadata, sort_keys=True))
        path.parent.mkdir(parents=True, exist_ok=True)
        with path.open('xb') as stream:
            np.savez_compressed(stream, **arrays)
        return dict(path=str(path), bytes=path.stat().st_size,
                    sha256=hashlib.sha256(path.read_bytes()).hexdigest(),
                    substep_rows=len(arrays['time_s']), contact_rows=len(arrays['contact_rows']), metadata=metadata)
