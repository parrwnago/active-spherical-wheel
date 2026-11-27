import numpy as np
from scipy.optimize import minimize,Bounds,LinearConstraint
from spherical_wheel.handoff_control import BoundedHandoffAllocator,HandoffSettings,_enumerated_equality_lsq
rng=np.random.default_rng(5602);caps=np.array([1.5,.6,.6,.3,.3]);worst=0.;recover=0
for i in range(90):
 G=.08*rng.normal(size=(9,5))
 if i%3==0:G[3:5]=0
 if i%3==1:G[4]=1.7*G[3]
 a=BoundedHandoffAllocator();axis=rng.normal(size=3);axis/=np.linalg.norm(axis)
 R=np.linalg.qr(rng.normal(size=(3,3)))[0];R[:,-1]*=np.linalg.det(R)
 old=rng.uniform(-1,1,5)*caps
 v=rng.normal(size=9);v[6:]*=12
 decision=a.solve(rng.normal(size=5)*caps*4,caps,old,.005,v,G,R,axis,[.0008,.0009],rng.uniform(0,12,4),rng.normal(size=2),readiness=rng.uniform(0,1,5),preparation_shell_rates=rng.normal(size=2)*10)
 d=decision.diagnostics;u=decision.command
 assert np.all(abs(u)<=caps+1e-12)
 assert np.all(abs(u-old)<=np.asarray(a.settings.slew_limits)*.005+1e-12)
 assert np.max(abs(d['secondary_balance_delta']),initial=0)<=1e-8
 excess=d['secondary_cost']-d['primary_point_secondary_cost'];worst=max(worst,excess)
 assert excess<=1e-7*max(1.,abs(d['primary_point_secondary_cost'])),(i,excess)
 recover+=int(d['secondary_recovery_used'])
for i in range(35):
 A=rng.normal(size=(10,5));b=rng.normal(size=10);E=np.linalg.qr(rng.normal(size=(5,2)))[0].T
 lo=np.full(5,-1.);hi=np.ones(5);seed=rng.uniform(-.9,.9,5);f=E@seed
 x,faces=_enumerated_equality_lsq(A,b,E,f,lo,hi,seed,1e-8)
 ref=minimize(lambda z:np.linalg.norm(A@z-b)**2,seed,jac=lambda z:2*A.T@(A@z-b),method='SLSQP',bounds=Bounds(lo,hi),constraints=LinearConstraint(E,f,f),options={'ftol':1e-12,'maxiter':100})
 assert ref.success
 assert np.max(abs(E@x-f))<1e-8
 assert np.linalg.norm(A@x-b)**2<=np.linalg.norm(A@ref.x-b)**2+1e-7
print({'allocator_random_scenarios':90,'enumeration_vs_independent_SLSQP':35,'max_secondary_cost_excess':worst,'recoveries':recover,'physics_executions':0})
