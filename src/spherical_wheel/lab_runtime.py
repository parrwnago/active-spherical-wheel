"""Interactive experiments using the real nonlinear plant and logged external pushes."""
from __future__ import annotations

from collections import deque
import copy
import csv
import io
import json
from pathlib import Path
import threading
import uuid

import mujoco
import numpy as np
import yaml

from .control import design_lqr, find_equilibrium, jsonable
from .experiments import rolling_feedforward
from .model import REPO_ROOT, generate_model, load_config
from .sim import Plant


class SimulationLab:
    """One serialized simulation session; wall-clock scheduling belongs to the server."""

    scenarios = {'shell': .30, 'polar': np.pi/2, 'transition': 1.10}

    def __init__(self, config=None, model_path=None):
        self.lock = threading.RLock()
        self.config = copy.deepcopy(config if isinstance(config, dict) else load_config(config))
        self.model_path = Path(model_path) if model_path else generate_model(self.config, REPO_ROOT/'generated'/'lab')
        self.plant = Plant(self.config, self.model_path)
        self.designer = Plant(self.config, self.model_path)
        self.dt = self.plant.dt
        self._design_cache = {}
        self.camera = {'azimuth': 135., 'elevation': -23., 'distance': .90}
        self.controller_enabled = True
        self.belt_enabled = True
        self.running = False
        self.failed = False
        self.last_error = None
        self._reset('shell', (0., 0.))

    def _design(self, phase):
        key = round(float(phase), 6)
        if key not in self._design_cache:
            equilibrium = find_equilibrium(self.designer, phase)
            design = design_lqr(self.designer, equilibrium)
            if design['full_closed_spectral_radius'] > 1.0001:
                raise RuntimeError('Local feedback has a growing full-state mode')
            belt_load = sum(c['normal_force_N'] for c in equilibrium['contacts'] if any(n.startswith('belt_') for n in c['geoms']))
            shell_load = sum(c['normal_force_N'] for c in equilibrium['contacts'] if any(n.startswith('shell_') for n in c['geoms']))
            self._design_cache[key] = {'phase': float(phase), 'equilibrium': equilibrium, 'design': design,
                                       'regime': 'belt' if belt_load > shell_load else 'shell'}
        return self._design_cache[key]

    def _reset(self, scenario, tilt):
        if scenario not in self.scenarios:
            raise ValueError('Choose shell, polar, or transition orientation')
        tilt = self._vector(tilt, 2, 10., 'Initial tilt')
        selected = self._design(self.scenarios[scenario])
        self.schedule = [selected]
        self.failed_gain_points = []
        if scenario == 'transition':
            self.schedule = []
            for phase in [1.10, 1.20, 1.28, 1.32, 1.40, 1.50, 1.57, 1.65]:
                try:
                    self.schedule.append(self._design(phase))
                except (RuntimeError, np.linalg.LinAlgError) as error:
                    self.failed_gain_points.append({'phase_rad': phase, 'error': str(error)})
            if not all(any(p['regime'] == regime for p in self.schedule) for regime in ('shell','belt')):
                raise RuntimeError('No usable gain schedule for both contact regimes')
        self.selected = selected
        eq = selected['equilibrium']
        self.plant.reset(qpos=eq['qpos'], qvel=eq['qvel'])
        perturbation = np.zeros(2*self.plant.model.nv)
        perturbation[3:5] = np.deg2rad(tilt)
        self.plant.perturb(perturbation)
        self.scenario = scenario
        self.initial_tilt_deg = tilt.tolist()
        self.running = False
        self.failed = False
        self.last_error = None
        self.run_id = uuid.uuid4().hex[:12]
        self.samples = deque(maxlen=120000)
        self.history = deque(maxlen=600)
        self.events = []
        self.dropped_samples = 0
        self.pulses = []
        self.last_push = None
        self.recovery_time_s = None
        self._recovery_candidate = None
        self._last_force = np.zeros(3)
        self._last_input = np.zeros(5)
        self._last_command = np.zeros(5)
        self._last_ref = eq['qpos'].copy()
        self._last_vref = np.zeros(self.plant.model.nv)
        self.nominal_joints = eq['qpos'][7:10].copy()
        self.anchor = self.plant.data.qpos[:2].copy()
        self.trajectory = {'kind': 'hold', 'duration_s': 10., 'distance_m': .04, 'lateral_m': .02, 'start_s': 0.}
        if scenario == 'transition':
            self.trajectory.update(kind='transition', distance_m=.06, lateral_m=.005)
        self._record(0.)
        self.events.append({'time_s': 0., 'action': 'reset', 'scenario': scenario, 'initial_tilt_deg': self.initial_tilt_deg})

    @staticmethod
    def _number(value, lower, upper, name):
        if isinstance(value, bool):
            raise ValueError(f'{name} must be a number')
        try:
            number = float(value)
        except (TypeError, ValueError):
            raise ValueError(f'{name} must be a finite number') from None
        if not np.isfinite(number) or not lower <= number <= upper:
            raise ValueError(f'{name} must be between {lower:g} and {upper:g}')
        return number

    @staticmethod
    def _vector(value, length, bound, name):
        try:
            vector = np.asarray(value, float)
        except (TypeError, ValueError):
            raise ValueError(f'{name} must contain {length} finite numbers') from None
        if vector.shape != (length,) or not np.all(np.isfinite(vector)) or np.any(abs(vector)>bound):
            raise ValueError(f'{name} must contain {length} finite values within ±{bound:g}')
        return vector

    def _reference(self):
        trajectory = self.trajectory
        elapsed = max(0., float(self.plant.data.time)-trajectory['start_s'])
        r = np.clip(elapsed/trajectory['duration_s'], 0., 1.)
        s = 10*r**3-15*r**4+6*r**5
        ds = (30*r**2-60*r**3+30*r**4)/trajectory['duration_s']
        dds = (60*r-180*r**2+120*r**3)/trajectory['duration_s']**2
        distance, lateral = trajectory['distance_m'], trajectory['lateral_m']
        if trajectory['kind'] == 'hold':
            return self.anchor.copy(), np.zeros(2), np.zeros(2)
        if trajectory['kind'] == 'sideways':
            return self.anchor+np.array([0.,distance*s]), np.array([0.,distance*ds]), np.array([0.,distance*dds])
        if trajectory['kind'] == 'line':
            return self.anchor+np.array([distance*s,0.]), np.array([distance*ds,0.]), np.array([distance*dds,0.])
        xy = np.array([distance*s,lateral*np.sin(np.pi*s)])
        derivative = np.array([distance,lateral*np.pi*np.cos(np.pi*s)])
        second = np.array([0.,-lateral*np.pi**2*np.sin(np.pi*s)])
        return self.anchor+xy, derivative*ds, second*ds**2+derivative*dds

    def _select_gain(self):
        if len(self.schedule) == 1:
            return
        forces = {'shell': 0., 'belt': 0.}
        for contact in self.plant.contact_summary():
            for name in contact['geoms']:
                if name.startswith('shell_'): forces['shell'] += contact['normal_force_N']
                elif name.startswith('belt_'): forces['belt'] += contact['normal_force_N']
        if sum(forces.values()) > .01:
            regime = max(forces, key=forces.get)
            candidates = [point for point in self.schedule if point['regime'] == regime]
            if candidates:
                self.selected = min(candidates,key=lambda point:abs(point['phase']-self.plant.data.qpos[7]))

    def _prepare_path_schedule(self, kind, distance):
        """Cover the requested local path, including a possible exit from a pole."""
        start = float(self.plant.data.qpos[7])
        end = start if kind in ('hold','sideways') else start+distance/self.config['geometry']['sphere_radius_m']
        count = max(2,int(np.ceil(abs(end-start)/.12))+1)
        phases = set(np.linspace(start,end,count).tolist())
        phases.add(self.selected['phase'])
        alpha = np.deg2rad(self.config['geometry']['polar_cap_angle_deg'])
        for boundary in (np.pi/2-alpha,np.pi/2+alpha):
            if min(start,end)-.05 <= boundary <= max(start,end)+.05:
                phases.update([boundary-.10,boundary+.10])
        available = []
        for phase in sorted(phases):
            try:
                available.append(self._design(phase))
            except (RuntimeError,np.linalg.LinAlgError) as error:
                self.failed_gain_points.append({'phase_rad':float(phase),'error':str(error)})
        if available:
            self.schedule=available

    def _advance_one(self):
        if self.failed:
            return
        self._select_gain()
        xy, velocity, acceleration = self._reference()
        eq, design = self.selected['equilibrium'], self.selected['design']
        qref = eq['qpos'].copy()
        qref[:2] = xy
        qref[7:10] = self.nominal_joints
        delta = np.zeros(self.plant.model.nv)
        gravity = abs(self.config['simulation']['gravity_m_s2'][2])
        delta[3:5] = [-np.arctan2(acceleration[1],gravity),np.arctan2(acceleration[0],gravity)]
        mujoco.mj_integratePos(self.plant.model,qref,delta,1.)
        vref, feedforward = rolling_feedforward(self.plant,velocity) if self.trajectory['kind']!='hold' else (np.zeros(9),np.zeros(5))
        command = eq['u']+feedforward-design['K']@self.plant.error(qref,vref) if self.controller_enabled else np.zeros(5)
        applied = np.clip(command,-self.plant.bounds,self.plant.bounds)
        if not self.belt_enabled:
            applied[3:] = 0.
        now = float(self.plant.data.time)
        self.pulses = [pulse for pulse in self.pulses if pulse['end_s'] > now+1e-10]
        force = sum((pulse['force_N'] for pulse in self.pulses if pulse['start_s'] <= now),start=np.zeros(3))
        self.plant.step(applied,np.r_[force,np.zeros(3)])
        self.nominal_joints += self.dt*vref[6:9]
        self._last_ref, self._last_vref = qref, vref
        self._last_command, self._last_input, self._last_force = command.copy(), applied.copy(), force.copy()
        snapshot = self.plant.snapshot()
        self.failed = bool(snapshot['balance_failed'] or snapshot['tilt_rad']>np.deg2rad(70) or not np.all(np.isfinite(snapshot['qpos'])))
        if self.failed:
            self.running = False
            self.events.append({'time_s':float(self.plant.data.time),'action':'fall','floor_contact':snapshot['balance_failed']})
        if self.last_push is not None and float(self.plant.data.time) >= self.last_push['time_s']+self.last_push['duration_s']:
            stable = snapshot['tilt_rad']<np.deg2rad(.5) and np.linalg.norm(snapshot['qvel'][3:6])<.08
            if stable and self._recovery_candidate is None:
                self._recovery_candidate = float(self.plant.data.time)
            elif not stable:
                self._recovery_candidate = None
            if self._recovery_candidate is not None and float(self.plant.data.time)-self._recovery_candidate>=.3 and self.recovery_time_s is None:
                self.recovery_time_s = max(0.,self._recovery_candidate-self.last_push['time_s']-self.last_push['duration_s'])
        self._record(now)

    def advance(self, steps=1):
        """Advance explicit controller intervals, including when paused for single-step."""
        with self.lock:
            if not isinstance(steps,int) or not 1<=steps<=2000:
                raise ValueError('Step count must be an integer from 1 to 2000')
            for _ in range(steps):
                if self.failed: break
                self._advance_one()

    def tick(self):
        with self.lock:
            if self.running and not self.failed:
                self._advance_one()

    def _record(self, interval_start):
        snapshot = self.plant.snapshot()
        quat = self.plant.data.qpos[3:7]
        roll = np.arctan2(2*(quat[0]*quat[1]+quat[2]*quat[3]),1-2*(quat[1]**2+quat[2]**2))
        pitch = np.arcsin(np.clip(2*(quat[0]*quat[2]-quat[3]*quat[1]),-1,1))
        sample = {'time_s':float(self.plant.data.time),'interval_start_s':float(interval_start),
                  'qpos':self.plant.data.qpos.copy(),'qvel':self.plant.data.qvel.copy(),
                  'reference_xy':self._last_ref[:2].copy(),'tilt_deg':float(np.rad2deg(snapshot['tilt_rad'])),
                  'roll_deg':float(np.rad2deg(roll)),'pitch_deg':float(np.rad2deg(pitch)),
                  'command':self._last_command.copy(),'input':self._last_input.copy(),'force_N':self._last_force.copy(),
                  'controller_enabled':self.controller_enabled,'belt_enabled':self.belt_enabled}
        if len(self.samples)==self.samples.maxlen: self.dropped_samples+=1
        self.samples.append(sample)
        if not self.history or sample['time_s']-self.history[-1]['time_s']>=.049:
            self.history.append(sample)

    def _status(self):
        if self.failed: return 'Fallen — reset to try again'
        if not self.running: return 'Paused'
        if not self.controller_enabled: return 'Controller off'
        if self.pulses: return 'Push applied'
        if self.samples[-1]['tilt_deg']>.5: return 'Recovering'
        if self.trajectory['kind']!='hold' and self.plant.data.time-self.trajectory['start_s']<self.trajectory['duration_s']: return 'Tracking'
        return 'Balanced'

    def state(self):
        with self.lock:
            sample = self.samples[-1]
            contacts = []
            for contact in self.plant.contact_summary():
                for name in contact['geoms']:
                    if name!='floor' and contact['normal_force_N']>.001:
                        contacts.append({'name':name,'normal_force_N':contact['normal_force_N']})
            trajectory = dict(self.trajectory)
            trajectory.pop('start_s')
            trajectory['elapsed_s']=max(0.,float(self.plant.data.time)-self.trajectory['start_s'])
            return {'time_s':sample['time_s'],'running':self.running,'failed':self.failed,'scenario':self.scenario,
                    'controller_enabled':self.controller_enabled,'belt_enabled':self.belt_enabled,'status':self._status(),
                    'tilt_deg':sample['tilt_deg'],'roll_deg':sample['roll_deg'],'pitch_deg':sample['pitch_deg'],
                    'position_xy_m':sample['qpos'][:2].tolist(),'reference_xy_m':sample['reference_xy'].tolist(),
                    'tracking_error_m':float(np.linalg.norm(sample['qpos'][:2]-sample['reference_xy'])),
                    'inputs':sample['input'].tolist(),'input_bounds':self.plant.bounds.tolist(),
                    'contacts':contacts,'active_force_N':self._last_force.tolist() if self.running else [0.,0.,0.],
                    'last_push':copy.deepcopy(self.last_push),'recovery_time_s':self.recovery_time_s,
                    'history':{'time_s':[point['time_s'] for point in self.history],
                               'tilt_deg':[point['tilt_deg'] for point in self.history],
                               'xy':[point['qpos'][:2].tolist() for point in self.history],
                               'reference_xy':[point['reference_xy'].tolist() for point in self.history],
                               'force_N':[point['force_N'].tolist() for point in self.history]},
                    'trajectory':trajectory,'run_id':self.run_id,'last_error':self.last_error,
                    'sample_count':len(self.samples),'dropped_samples':self.dropped_samples,
                    'gain_points_rejected':copy.deepcopy(self.failed_gain_points),
                    'units':{'position':'m','force':'N','joint_torques':'N m','belt_speed':'m/s'},
                    'camera':dict(self.camera)}

    def command(self, payload):
        with self.lock:
            if not isinstance(payload,dict): raise ValueError('Command must be a JSON object')
            action = payload.get('action')
            now = float(self.plant.data.time)
            if action=='reset':
                self._reset(payload.get('scenario','shell'),payload.get('initial_tilt_deg',[0.,0.]))
            elif action=='resume':
                if self.failed: raise ValueError('Reset the fallen model before resuming')
                self.running=True
            elif action=='pause': self.running=False
            elif action=='step':
                self.running=False
                self.advance()
            elif action in ('controller','belt'):
                if not isinstance(payload.get('enabled'),bool): raise ValueError('enabled must be true or false')
                setattr(self,'controller_enabled' if action=='controller' else 'belt_enabled',payload['enabled'])
            elif action=='push':
                if self.failed: raise ValueError('Reset the fallen model before pushing')
                force = self._vector(payload.get('force_N'),3,10.,'Force')
                if np.linalg.norm(force)>10.: raise ValueError('Force magnitude must not exceed 10 N')
                duration = self._number(payload.get('duration_s',.1),.02,1.,'Pulse duration')
                if len(self.pulses)>=16: raise ValueError('Too many overlapping pushes')
                self.pulses.append({'start_s':now,'end_s':now+duration,'force_N':force})
                self.last_push={'force_N':force.tolist(),'duration_s':duration,'time_s':now}
                self.recovery_time_s=None;self._recovery_candidate=None
                self.events.append({'time_s':now,'action':'push',**copy.deepcopy(self.last_push)})
            elif action=='trajectory':
                kind = payload.get('kind','hold')
                if kind not in ('hold','line','sideways','curve','transition'): raise ValueError('Unknown trajectory')
                duration = self._number(payload.get('duration_s',10.),5.,60.,'Trajectory duration')
                distance = self._number(payload.get('distance_m',.06 if kind=='transition' else .04),-.10,.10,'Distance')
                lateral = self._number(payload.get('lateral_m',.005 if kind=='transition' else .02),-.05,.05,'Lateral excursion')
                if kind=='transition':
                    was_running=self.running
                    self._reset('transition',[0.,0.])
                    self.running=was_running
                    now=float(self.plant.data.time)
                elif kind!='hold':
                    self._prepare_path_schedule(kind,distance)
                self.anchor=self.plant.data.qpos[:2].copy()
                self.nominal_joints=self.plant.data.qpos[7:10].copy()
                self.trajectory={'kind':kind,'duration_s':duration,'distance_m':distance,'lateral_m':lateral,'start_s':now}
                self.events.append({'time_s':now,'action':'trajectory',**self.trajectory})
            elif action=='camera':
                for name,lower,upper in [('azimuth',-360.,360.),('elevation',-85.,-5.),('distance',.5,3.)]:
                    if name in payload: self.camera[name]=self._number(payload[name],lower,upper,name)
            elif action=='save':
                path=self.save_run()
                return {'ok':True,'directory':str(path),'download_url':'/api/export','state':self.state()}
            else: raise ValueError('Unknown command action')
            if action not in ('push','reset','trajectory'):
                self.events.append({'time_s':now,'action':action,**{key:value for key,value in payload.items() if key!='action'}})
            return {'ok':True,'state':self.state()}

    def render_snapshot(self):
        with self.lock:
            return {'qpos':self.plant.data.qpos.copy(),'time_s':float(self.plant.data.time),
                    'reference_xy_m':self._last_ref[:2].copy(),
                    'path_xy_m':np.array([point['qpos'][:2] for point in self.history]),
                    'reference_path_xy_m':np.array([point['reference_xy'] for point in self.history]),
                    'force_N':self._last_force.copy() if self.running else np.zeros(3),
                    'camera':dict(self.camera),'status':self._status(),'run_id':self.run_id}

    def export_csv(self):
        with self.lock:
            output=io.StringIO()
            writer=csv.writer(output)
            writer.writerow(['time_s','interval_start_s','x_m','y_m','reference_x_m','reference_y_m','roll_deg','pitch_deg','tilt_deg',
                             'force_x_N','force_y_N','force_z_N','shaft_torque_Nm','shell_A_torque_Nm','shell_B_torque_Nm',
                             'belt_A_speed_m_s','belt_B_speed_m_s','controller_enabled','belt_enabled',
                             'requested_shaft_torque_Nm','requested_shell_A_torque_Nm','requested_shell_B_torque_Nm',
                             'requested_belt_A_speed_m_s','requested_belt_B_speed_m_s'])
            for sample in self.samples:
                writer.writerow([sample['time_s'],sample['interval_start_s'],*sample['qpos'][:2],*sample['reference_xy'],
                                 sample['roll_deg'],sample['pitch_deg'],sample['tilt_deg'],*sample['force_N'],*sample['input'],
                                 sample['controller_enabled'],sample['belt_enabled'],*sample['command']])
            return output.getvalue()

    def save_run(self):
        with self.lock:
            path=REPO_ROOT/'results'/'live'/self.run_id
            path.mkdir(parents=True,exist_ok=True)
            generate_model(self.config,path/'model')
            arrays={key:np.asarray([sample[key] for sample in self.samples]) for key in ('time_s','qpos','qvel','command','input','force_N','reference_xy','tilt_deg')}
            arrays['disturbance']=np.c_[arrays['force_N'],np.zeros((len(self.samples),3))]
            arrays['tilt_rad']=np.deg2rad(arrays['tilt_deg'])
            arrays['model_path']=np.array('model/model.xml')
            np.savez_compressed(path/'rollout.npz',**arrays)
            (path/'run.csv').write_text(self.export_csv())
            (path/'configuration.yaml').write_text(yaml.safe_dump(self.config,sort_keys=False))
            (path/'events.json').write_text(json.dumps(jsonable(self.events),indent=2))
            (path/'summary.json').write_text(json.dumps({'run_id':self.run_id,'engine_version':mujoco.__version__,'scenario':self.scenario,
                                                        'trajectory':self.trajectory,'retained_samples':len(self.samples),'dropped_samples':self.dropped_samples,
                                                        'failed':self.failed,'initial_tilt_deg':self.initial_tilt_deg,
                                                        'interval_alignment':'Logged inputs/forces produced state at time_s from interval_start_s.',
                                                        'limits':'Assumed masses/friction; ideal belt speed servos; flat-floor contact.'},indent=2))
            return path
