from __future__ import annotations
import argparse
import json
from pathlib import Path


def main(argv=None):
    parser=argparse.ArgumentParser(description='Active spherical wheel simulation and local feedback')
    sub=parser.add_subparsers(dest='command',required=True)
    for name in ['generate','validate','analyze','balance','track','zero','viewer']:
        p=sub.add_parser(name)
        p.add_argument('--config',type=Path)
        p.add_argument('--output',type=Path,default=Path('generated') if name=='generate' else Path('results')/name)
        p.add_argument('--duration',type=float,default=8. if name=='track' else 5.)
        p.add_argument('--phase',type=float)
        p.add_argument('--tilt',type=float,nargs=2,default=[2.,2.],metavar=('ROLL_DEG','PITCH_DEG'))
        p.add_argument('--method',choices=['full_state_lqr','frozen_phase_feedback'],default='full_state_lqr')
        p.add_argument('--disturbance',type=float,default=0.,help='Horizontal force in N from1.0 to1.1s')
    p=sub.add_parser('render');p.add_argument('--log',type=Path,required=True);p.add_argument('--output',type=Path,required=True);p.add_argument('--fps',type=int,default=30)
    for name in ('transition','sensitivity'):
        p=sub.add_parser(name)
        p.add_argument('--output',type=Path,default=Path('results')/name)
        p.add_argument('--duration',type=float,default=10. if name=='transition' else 5.)
    p=sub.add_parser('lab',help='Live browser controls for pushes, polar balance and trajectories')
    p.add_argument('--host',default='127.0.0.1')
    p.add_argument('--port',type=int,default=8765)
    p.add_argument('--open-browser',action='store_true')
    p.add_argument('--controller-mode',choices=['transition_aware','local_lqr'],default='transition_aware')
    p=sub.add_parser('offroad',help='Uneven-ground runs, contact-force analysis and controller tuning')
    p.add_argument('action',choices=['limits','run','replay','suite','envelope','holdout','tune'])
    p.add_argument('--speed',type=float,default=2.,help='Playback speed of the replay action')
    p.add_argument('--config',type=Path)
    p.add_argument('--output',type=Path,default=Path('results')/'offroad')
    p.add_argument('--scenario',default='course',help='Scenario name for the run action')
    p.add_argument('--controller',choices=['terrain_aware','momentum_observer','flat_baseline'],default='terrain_aware',
                   help='momentum_observer: terrain_aware without the solved contact force (no force sensor)')
    p.add_argument('--seed',type=int,default=7)
    p.add_argument('--workers',type=int,help='Worker processes for batch actions')
    p.add_argument('--view-time',type=float,help='Time in s of the rendered still for the run action (default: midway)')
    args=parser.parse_args(argv)
    if args.command=='offroad':
        from .offroad_cli import main as offroad_main
        offroad_main(args);return
    if args.command=='lab':
        from .lab_server import run_server
        run_server(args.host,args.port,args.open_browser,args.controller_mode);return
    if args.command in ('transition','sensitivity'):
        if args.command=='transition':
            from .transition import run_transition
            result=run_transition(args.output,args.duration)
        else:
            from .sensitivity import run_sensitivity
            result=run_sensitivity(args.output,args.duration)
        print(json.dumps(result,indent=2));return
    if args.command=='render':
        from .visualize import render_rollout
        render_rollout(args.log,args.output,fps=args.fps)
        print(args.output);return
    from .model import load_config,generate_model
    cfg=load_config(args.config)
    if args.phase is None:
        args.phase=cfg['simulation'].get('initial_shaft_phase_rad',.30)
    if args.command=='generate':
        print(generate_model(cfg,args.output));return
    if args.command=='validate':
        from .validation import run_physics_validation
        path=generate_model(cfg)
        print(json.dumps(run_physics_validation(path,args.output,config=cfg),indent=2));return
    if args.command=='viewer':
        from .visualize import view
        view(generate_model(cfg));return
    if args.command=='analyze':
        from .sim import Plant
        from .control import save_design
        eq,d=save_design(Plant(cfg),args.output,args.phase,args.method)
        print(json.dumps({'equilibrium_residual_norm':float(__import__('numpy').linalg.norm(eq['step_residual'])),'reachable_rank':d['reachable_rank'],'full_closed_spectral_radius':d['full_closed_spectral_radius']},indent=2));return
    from .experiments import run_experiment
    result=run_experiment(args.command,args.output,args.duration,args.tilt,cfg,args.phase,args.method,args.disturbance)
    print(json.dumps(result,indent=2))


if __name__=='__main__':main()
