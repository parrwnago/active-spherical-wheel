"""Balance and tracking feedback that uses the measured ground reaction.

The flat-floor controller assumes the ground normal is world-vertical. On a
slope that assumption is wrong in several places. Each remedy needs only the
contact force and moment at the wheel (``contact_sensing``). They are listed
by how much the tuning sweep found they matter (results/offroad/tuning):

1. **Where to lean.** A wheel standing on a grade touches the ground uphill of
   its centre, so the whole machine must lean until its centre of mass is over
   that contact point. For this wheel the lean is almost as large as the grade
   itself. Without it the position loop has to supply the lean as a standing
   error, roughly 0.7 m of sag on an 8 degree grade. This is the one that
   decides the outcome: a lean 10% too small or too large is measurably worse.
2. **What torque holds it there.** The weight acting through the offset
   contact point loads the shaft and the loaded shell. That static torque is
   computed from the sensed normal and fed forward, so feedback only handles
   deviations. Removing it costs less than removing the lean.
3. **Shaft angle that agrees with position.** Rolling without slip ties
   position, pitch and shaft angle together. Distance travelled along
   undulating ground breaks an integrated shaft-angle reference, so the shaft
   error is taken from the position error instead. A small effect.
4. **Whether it is on the ground at all.** A handover between surfaces can
   leave one controller interval with no contact. The scheduled gain, the
   load shares and the rolling reference of the last loaded interval are kept
   through it. Torques also fade if the filtered normal load stays below a
   threshold; no run in the suite ever reached it.

Measuring the gain schedule's phase against the sensed normal instead of
vertical is available (``schedule_on_ground_normal``) and off: the sweep found
no benefit from it.

Two things were tried and are deliberately absent. Scaling torque to keep the
implied tangential force inside an estimated friction cone made the wheel
fall: a balancing machine's holding torque is not optional, and cutting it to
protect traction lets the body topple. The friction estimate is therefore
reported but does not limit any command. Freezing the sensed normal while the
support crosses the equatorial gap left it stale whenever the wheel lingered
there; it is slowed instead.

The feedback gains themselves are the unchanged flat-floor library. Nothing
here adds an external force, resets a state or moves the requested target.
State feedback and the contact wrench are simulator ground truth.
"""
from __future__ import annotations

from dataclasses import asdict, dataclass

import mujoco
import numpy as np

from .contact_sensing import ContactEstimator, measure_contact
from .control import design_lqr, find_equilibrium
from .experiments import rolling_feedforward
from .recovery_control import ScheduledBalanceFeedback, build_gain_library
from .synchronization import ContactSynchronizer


@dataclass
class OffroadParams:
    """Controller settings. ``flat_baseline`` ignores every terrain option.

    The defaults are the outcome of the tuning sweep saved in
    results/offroad/tuning/tuning.json.
    """
    mode: str = 'terrain_aware'            # 'terrain_aware' or 'flat_baseline'
    synchronize: bool = True               # prepare incoming drives, as in the lab
    normal_source: str = 'intrinsic'       # 'intrinsic' (force and moment) or 'solver'
    normal_time_constant_s: float = .02    # low-pass on the sensed ground normal
    rim_aware: bool = True                 # treat the equatorial rims as rims; False reads them as sphere
    rim_time_constant_s: float = .2        # low-pass along the carrier axis while the support is on the rims
    rim_hold_speed_m_s: float = .02        # resting on both rims below this speed, the axial part is held
    gap_margin_m: float = .002             # treat this far beyond the gap as rim support too
    lean_gain: float = 1.                  # 1 puts the centre of mass over the contact point
    support_feedforward: float = 1.        # 1 applies the full static holding torque
    schedule_on_ground_normal: bool = False  # True measures the gain phase against the sensed normal
    shaft_error: str = 'rolling'           # 'material', 'free' or 'rolling'
    track_contact_point: bool = False      # reference applies to the ground contact point, not the wheel centre
    slip_threshold_m_s: float = .02        # surface speed that counts as sliding for the friction estimate
    friction_recovery_s: float = 1.5       # how fast that estimate relaxes back once sliding stops
    heading_gain: float = 0.               # N m of yaw reaction per rad of heading error; 0 leaves heading free
    heading_damping: float = .06           # N m s/rad on yaw rate
    heading_torque_limit_Nm: float = .15   # bound on the idle shell's heading torque
    heading_min_lever: float = .12         # skip the hold when the shell axis is this close to horizontal
    unload_fraction: float = .3            # below this share of weight, feedback torque fades out
    airborne_damping_Nms: float = .01      # relative-speed damping while unloaded
    max_lean_deg: float = 28.              # bound on the reference lean

    def __post_init__(self):
        if self.mode not in ('terrain_aware', 'flat_baseline'):
            raise ValueError('Choose terrain_aware or flat_baseline')
        if self.shaft_error not in ('material', 'free', 'rolling'):
            raise ValueError('Shaft error must be material, free or rolling')
        if self.normal_source not in ('intrinsic', 'solver'):
            raise ValueError('Normal source must be intrinsic or solver')

    def as_dict(self):
        return asdict(self)


class FlatDesign:
    """The flat-floor gain library and local designs, built once and shared.

    Uses a private flat ``Plant`` for design, exactly as the interactive lab
    does, so the terrain plant is never perturbed by a design calculation.
    """

    def __init__(self, designer, cache_path=None):
        self.designer = designer
        self._local = {}
        self.points, self.metadata = build_gain_library(designer, None, cache_path)

    def local(self, phase):
        key = round(float(phase), 6)
        if key not in self._local:
            equilibrium = find_equilibrium(self.designer, phase)
            design = design_lqr(self.designer, equilibrium)
            if design['full_closed_spectral_radius'] > 1.0001:
                raise RuntimeError('Local feedback has a growing full-state mode')
            self._local[key] = {'phase': float(phase), 'equilibrium': equilibrium, 'design': design}
        return self._local[key]


def _quat_from_rotation_vector(vector):
    angle = float(np.linalg.norm(vector))
    if angle < 1e-12:
        return np.array([1., 0., 0., 0.])
    return np.r_[np.cos(angle/2), np.sin(angle/2)*np.asarray(vector)/angle]


class OffroadController:
    """Five drive commands from the plant state and the measured ground reaction.

    ``command(xy, velocity, acceleration)`` returns ``(applied, diagnostics)``
    where ``applied`` is already inside the actuator bounds. Call
    ``after_step()`` once the plant has advanced. ``reset(ordinary)`` selects
    the ordinary operating point whose equilibrium and shaft-row gain anchor
    the schedule, as the lab's scenario reset does.
    """

    def __init__(self, plant, design, params=None, phase=None):
        self.plant, self.design = plant, design
        self.params = params or OffroadParams()
        self.feedback = ScheduledBalanceFeedback(plant, design.points)
        self.synchronizer = ContactSynchronizer(plant)
        model = plant.model
        self.carrier_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, 'carrier')
        self.radius = float(plant.config['geometry']['sphere_radius_m'])
        self.gap_half = float(plant.config['geometry'].get('equatorial_gap_half_m', 0.))
        self.gravity = abs(float(plant.config['simulation']['gravity_m_s2'][2]))
        self.total_mass = float(model.body_subtreemass[plant.fork_id])
        self.weight = self.total_mass*self.gravity
        # Lean per unit of contact offset: total weight acting at the wheel
        # radius against the upper body's first moment about the wheel centre.
        upper_moment = float(model.body_mass[plant.fork_id]*model.body_ipos[plant.fork_id][2])
        self.lean_factor = self.total_mass*self.radius/upper_moment
        self.nominal_friction = float(plant.config['contact']['friction'][0])
        self.estimator = ContactEstimator(self.radius, self.weight, self.nominal_friction,
                                          self.params.normal_time_constant_s,
                                          slip_threshold_m_s=self.params.slip_threshold_m_s,
                                          friction_recovery_s=self.params.friction_recovery_s,
                                          source=self.params.normal_source,
                                          rim_time_constant_s=self.params.rim_time_constant_s)
        self.reset(phase)

    def reset(self, phase=None):
        phase = self.plant.config['simulation'].get('initial_shaft_phase_rad', .30) if phase is None else phase
        self.ordinary = self.design.local(phase)
        self.nominal_joints = self.ordinary['equilibrium']['qpos'][7:10].copy()
        ground = getattr(self.plant, 'ground', None)
        self.estimator.reset(ground['normal'] if ground else (0., 0., 1.))
        self._vref = np.zeros(self.plant.model.nv)
        self._sync = {'commands': [0.]*5, 'blend_weights': [0.]*5, 'targets': []}
        self._held = None
        quat = self.plant.data.qpos[3:7]
        self.heading_reference = float(np.arctan2(2*(quat[0]*quat[3]+quat[1]*quat[2]),
                                                  1-2*(quat[2]**2+quat[3]**2)))

    def _rim_support(self, loads):
        """``(axis, hold)`` while the support point lies in the equatorial gap, else ``(None, False)``.

        There the wheel rides on one or both shell rims, up to the half gap
        away from where a sphere would touch. The carrier angle against the
        current normal estimate locates the support, so this needs no extra
        sensing. ``gap_margin_m`` widens the band a little for the estimate's
        own lag. ``axis`` is the carrier's x axis, along which a rim displaces
        the contact. ``hold`` is true when the wheel is resting there on both
        rims, moving slower than ``rim_hold_speed_m_s``.
        """
        axis = self.plant.data.xmat[self.carrier_id].reshape(3, 3)[:, 0].copy()
        offset = self.radius*abs(float(axis@self.estimator.normal))
        if offset > self.gap_half+self.params.gap_margin_m:
            return None, False
        total = sum(loads.values())
        both = total > 0 and min(loads['shell_A'], loads['shell_B']) > .05*total
        resting = float(np.linalg.norm(self.plant.data.qvel[:2])) < self.params.rim_hold_speed_m_s
        return axis, bool(both and resting)

    # -- reference ----------------------------------------------------------
    def lean_vector(self, normal, acceleration):
        """Horizontal part of the desired body-up direction."""
        lean = np.asarray(acceleration, float)/self.gravity
        if self.params.mode == 'terrain_aware':
            lean = lean-self.params.lean_gain*self.lean_factor*np.asarray(normal)[:2]
        limit = np.sin(np.deg2rad(self.params.max_lean_deg))
        size = float(np.linalg.norm(lean))
        return lean*limit/size if size > limit else lean

    def reference_attitude(self, normal, acceleration):
        """Reference quaternion before heading is applied."""
        lean = self.lean_vector(normal, acceleration)
        size = float(np.linalg.norm(lean))
        vector = np.zeros(3) if size < 1e-12 else np.arcsin(size)*np.array([-lean[1], lean[0], 0.])/size
        quat = np.empty(4)
        mujoco.mju_mulQuat(quat, self.ordinary['equilibrium']['qpos'][3:7], _quat_from_rotation_vector(vector))
        return quat

    def support_torque(self, normal, loads):
        """Static shaft and shell torques that carry the weight through the sensed contact point."""
        data = self.plant.data
        shaft_axis = data.xmat[self.plant.fork_id].reshape(3, 3)[:, 1]
        shell_axis = data.xmat[self.carrier_id].reshape(3, 3)[:, 0]
        total = sum(loads.values())
        lever = self.weight*self.radius
        torque = np.zeros(5)
        torque[0] = lever*np.cross(shaft_axis, normal)[2]
        if total > 1e-6:
            shell = lever*np.cross(shell_axis, normal)[2]
            torque[1] = shell*loads['shell_A']/total
            torque[2] = shell*loads['shell_B']/total
        return torque

    # -- command --------------------------------------------------------------
    def command(self, xy, velocity, acceleration):
        plant, params = self.plant, self.params
        equilibrium, design = self.ordinary['equilibrium'], self.ordinary['design']
        measurement = measure_contact(plant)
        rim = self._rim_support(measurement['loads_N']) if params.rim_aware else (None, False)
        estimate = self.estimator.update(measurement, plant.dt, *rim)
        aware = params.mode == 'terrain_aware'
        normal = estimate['normal'] if aware else np.array([0., 0., 1.])
        moving = bool(np.linalg.norm(velocity) > 0)
        touching = sum(measurement['loads_N'].values()) > .001
        if not moving:
            vref, feedforward = np.zeros(9), np.zeros(5)
        elif aware and not touching and self._held is not None:
            # A surface handover can leave one interval with no contact. The
            # rolling allocation needs a contact, so keep the last one rather
            # than drop the velocity reference to zero for that interval.
            vref, feedforward = self._held['vref'].copy(), self._held['feedforward'].copy()
        else:
            vref, feedforward = rolling_feedforward(plant, np.asarray(velocity, float))
        qref = equilibrium['qpos'].copy()
        qref[:2] = xy
        if aware and params.track_contact_point:
            # The reference names where the wheel should touch the ground. The
            # centre sits one radius along the ground normal from that point,
            # so on a slope it is offset horizontally and need not be dragged
            # back over the reference as the slope comes and goes.
            qref[:2] = np.asarray(xy)+self.radius*normal[:2]
        qref[7:10] = self.nominal_joints
        qref[3:7] = self.reference_attitude(normal, acceleration)
        diagnostics = {'measurement': measurement, 'estimate': estimate, 'drive_fade': 1.,
                       'support_torque': np.zeros(5), 'reference_quat': qref[3:7].copy()}
        if aware:
            command, feedback = self._terrain_feedback(qref, vref, feedforward, normal, measurement, touching)
            diagnostics['support_torque'] = feedback.pop('support_torque')
        else:
            command, feedback = self.feedback.compute(qref, vref, equilibrium['u'], design['K'], feedforward)
        diagnostics.update(effective_phase_rad=feedback['effective_phase_rad'], yaw_rad=feedback['yaw_rad'],
                           error=feedback['error'])
        sync = {'commands': [0.]*5, 'blend_weights': [0.]*5, 'targets': []}
        if params.synchronize:
            # The preview owns a private MjData; give it the current ground plane.
            self.synchronizer._data.mocap_pos[:] = plant.data.mocap_pos
            self.synchronizer._data.mocap_quat[:] = plant.data.mocap_quat
            sync = self.synchronizer.preview(command)
            weights = np.asarray(sync['blend_weights'])
            command = (1.-weights)*command+weights*np.asarray(sync['commands'])
        if aware:
            heading_torque, diagnostics['heading_error_rad'] = self._heading_torque(feedback['yaw_rad'])
            command = command+heading_torque
            command, diagnostics['drive_fade'] = self._fade_when_unloaded(command, estimate, vref)
        self._sync, self._vref = sync, vref
        applied = np.clip(command, -plant.bounds, plant.bounds)
        diagnostics.update(requested=command.copy(), reference_qpos=qref, reference_qvel=vref,
                           sync_weights=np.asarray(sync['blend_weights']))
        return applied, diagnostics

    def _terrain_feedback(self, qref, vref, feedforward, normal, measurement, touching):
        plant, params = self.plant, self.params
        equilibrium, design = self.ordinary['equilibrium'], self.ordinary['design']
        rotation = plant.data.xmat[self.carrier_id].reshape(3, 3)
        support = rotation.T@(normal if params.schedule_on_ground_normal else np.array([0., 0., 1.]))
        phase = float(np.arctan2(-support[0], support[2]))
        if touching:
            loads = dict(measurement['loads_N'])
            gain = self.feedback._gain(phase, loads)
            self._held = {'loads': loads, 'gain': gain.copy(), 'vref': np.array(vref, copy=True),
                          'feedforward': np.array(feedforward, copy=True)}
        elif self._held is not None:
            # Unloaded for an interval: the surface that was carrying the wheel
            # is still the one under it. Falling back to the ordinary-pose gain
            # here would apply, say, shell gains while a belt is underneath.
            loads, gain = self._held['loads'], self._held['gain'].copy()
        else:
            loads, gain = dict(measurement['loads_N']), np.array(design['K'], copy=True)
        gain[0] = design['K'][0]
        quat = plant.data.qpos[3:7]
        yaw = float(np.arctan2(2*(quat[0]*quat[3]+quat[1]*quat[2]), 1-2*(quat[2]**2+quat[3]**2)))
        c, s = np.cos(yaw), np.sin(yaw)
        world_to_heading = np.array([[c, s], [-s, c]])
        target = np.array(qref, copy=True)
        mujoco.mju_mulQuat(target[3:7], np.array(qref[3:7]), np.array([np.cos(yaw/2), 0., 0., np.sin(yaw/2)]))
        # Follow the ground: its height and the vertical speed of travelling
        # along it are not errors, and no drive can act on them anyway.
        target[2] = plant.data.qpos[2]
        vref = np.array(vref, copy=True)
        vref[2] = -float(normal[:2]@vref[:2])/max(normal[2], .2)
        error = plant.error(target, vref)
        error[:2] = world_to_heading@error[:2]
        error[9:11] = world_to_heading@error[9:11]
        error[6:9] = (error[6:9]+np.pi) % (2*np.pi)-np.pi
        error[7:9] = 0.
        if params.shaft_error == 'free':
            error[6] = 0.
        elif params.shaft_error == 'rolling':
            # Rolling without slip ties x, pitch and shaft angle together:
            # x - R (pitch + shaft) is constant. Ground travelled along an
            # undulating surface breaks the flat bookkeeping of shaft angle, so
            # take the shaft error that is consistent with the position error.
            error[6] = error[0]/self.radius-error[4]
        support_torque = params.support_feedforward*self.support_torque(normal, loads)
        command = np.asarray(equilibrium['u'])+np.asarray(feedforward)+support_torque-gain@error
        return command, {'effective_phase_rad': phase, 'yaw_rad': yaw, 'error': error,
                         'support_torque': support_torque}

    def _heading_torque(self, yaw):
        """Hold the starting heading with whichever shell is not carrying load.

        A shell that is off the ground is a reaction wheel about the carrier's
        x axis. That axis is tilted out of the horizontal by the carrier phase,
        so the unloaded shell's motor reaction has a vertical component, and
        that is the only yaw authority a single point contact leaves. Rolling
        sideways spins the loaded shell about the same tilted axis and turns
        the platform the other way; counter-spinning the idle shell cancels
        it. Nothing is available when the axis is near horizontal, so the hold
        is skipped there, and when both shells are loaded.

        The reaction's horizontal part is a roll disturbance that the balance
        feedback has to absorb, which is why the torque is kept small.
        """
        params, plant = self.params, self.plant
        error = float((yaw-self.heading_reference+np.pi) % (2*np.pi)-np.pi)
        torque = np.zeros(5)
        if params.heading_gain <= 0 or self._held is None:
            return torque, error
        loads = self._held['loads']
        total = sum(loads.values())
        free = [index for index, name in ((1, 'shell_A'), (2, 'shell_B')) if loads[name] <= .05*total]
        vertical = float(plant.data.xmat[self.carrier_id].reshape(3, 3)[2, 0])
        if not free or abs(vertical) < params.heading_min_lever:
            return torque, error
        velocity = np.empty(6)
        mujoco.mj_objectVelocity(plant.model, plant.data, mujoco.mjtObj.mjOBJ_BODY, plant.fork_id, velocity, 0)
        wanted = -(params.heading_gain*error+params.heading_damping*velocity[2])
        # Motor torque t on a free shell reacts on the platform as -t * axis.
        each = float(np.clip(-wanted/vertical, -params.heading_torque_limit_Nm, params.heading_torque_limit_Nm))
        torque[free] = each/len(free)
        return torque, error

    def _fade_when_unloaded(self, command, estimate, vref):
        """Stop driving the wheel while it carries little or no load.

        Off the ground a torque only spins its drive up, and the wheel then
        lands sliding. Below ``unload_fraction`` of weight the torques fade and
        the drives are damped toward their rolling reference instead. The
        filtered load is used, so a single-interval dropout at a surface
        handover does not trigger it.
        """
        params, plant = self.params, self.plant
        fade = float(np.clip(estimate['load_fraction']/max(params.unload_fraction, 1e-6), 0., 1.))
        if fade >= 1.:
            return command, 1.
        command = np.array(command, copy=True)
        damping = -params.airborne_damping_Nms*(plant.data.qvel[6:9]-vref[6:9])
        command[:3] = fade*command[:3]+(1.-fade)*damping
        return command, fade

    def after_step(self):
        """Advance the material joint references, as the lab runtime does."""
        dt = self.plant.dt
        self.nominal_joints += dt*self._vref[6:9]
        for target in self._sync['targets']:
            if 'target_joint_speed_rad_s' in target:
                index = target['command_index']
                self.nominal_joints[index] += dt*target['blend_weight']*target['target_joint_speed_rad_s']
