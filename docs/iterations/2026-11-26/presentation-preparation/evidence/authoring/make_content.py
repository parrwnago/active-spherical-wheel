from pathlib import Path
import json
HERE=Path(__file__).resolve().parent
pages=[]
def P(text):return dict(type='p',text=text)
def H(text):return dict(type='h',text=text)
def B(*items):return dict(type='bullets',items=list(items))
def E(text,height=.47):return dict(type='eq',text=text,height=height)
def T(rows,widths=None):return dict(type='table',rows=rows,**({'widths':widths} if widths else {}))
def F(path,caption='',height=210):return dict(type='figure',path=path,caption=caption,height=height)
def C(text):return dict(type='callout',text=text)
def S(text):return dict(type='small',text=text)
def page(id,title,section,blocks,sources=''):
 pages.append(dict(id=id,title=title,section=section,blocks=blocks,sources=sources))
page('start','Understand the controller, then defend the results','Study guide • current selected simulation baseline',[
 P('Active spherical wheel: dynamics, gain-scheduled LQR, contact-aware feedforward, momentum-based contact estimation and incoming-drive preview. Prepared 26 November 2026 from the current implementation and saved experiment evidence.'),
 C('<b>The key correction:</b> the actuator bounds already limit the applied commands. A remaining design gap is consistent anticipation of those limits before a large correction is clipped; addressing that gap is not yet a proven drift cure. A penalty on effort is not a hard inequality constraint.'),
 H('What this guide establishes'),
 B('The 0.835 m/s belt request is mostly balance feedback in one inspected event; the desired body speed there is only about 0.045 m/s. These are different quantities.',
 'Some configurations lose sideways authority because the drive velocity directions align. Small tracking error must sometimes be accepted; cost tuning cannot create a missing direction.',
 'The saved lockstep tests do not demonstrate robot motion during Python computation. Filtering, preceding-interval information, patch boundaries and solver/state timing must be separated.',
 'Turning overshoot is measurable, but previous improvements came with tradeoffs. The selected LQR baseline remains unchanged.'),
 H('Scope'),
 P('This is a preparation document, not a five-minute slide deck. It explains the selected loop and re-examines existing logs. No new simulation or controller change was performed for this document. Cameras and deployment questions are outside this presentation scope.'),
 S('Evidence labels throughout: <b>Implemented</b> = code actually in the selected loop. <b>Observed</b> = saved trace or controlled comparison. <b>Interpretation</b> = explanation consistent with that evidence. <b>Proposed</b> = not an adopted fix.')
], 'Current code [C1–C9]; experiment records [E12, E41–E43, E46]. Reference and evidence indexes appear at the end.')
page('reading','How to use this guide','Preparation route',[
 P('Read the current-loop and constraint pages first. Then work through the observer and geometry. You should be able to explain one complete control interval without equations before practicing the mathematical questions.'),
 dict(type='toc',items=[('loop','1. What runs every 5 ms'),('coordinates','2. Coordinates, physical axes and input units'),('dynamics','3. The nonlinear dynamics being simulated'),('rolling','4. How desired motion becomes rolling references'),('lqr','5. What LQR optimizes'),('constraints','6. Why effort penalties do not impose hard limits'),('saturation','7. What happened in the saturated event'),('observer','8. The momentum observer, step by step'),('contact','9. From net wrench to contact geometry'),('timing','10. Test the lag hypothesis against the actual clocks'),('authority','11. Why drive directions can coincide'),('turns','12. Turning response and prior tradeoffs'),('next','13. What should be investigated next'),('qa1','14. Practice questions and concise answers'),('talk','15. A five-minute presentation outline'),('references','16. Citations, tools and contribution credit')]),
 H('Three terms to keep separate'),
 T([['Quantity','Meaning'],['Body velocity','Translation of the wheel center in the world, m/s.'],['Drive-rate reference','Desired shaft/shell angular rate, rad/s, or belt surface speed, m/s.'],['Applied input','Three joint torques plus two prescribed belt surface speeds. A joint-rate reference is not itself a torque command.']],[124,351]),
 P('Also distinguish tracking a timed trajectory from returning to a geometric path, and absolute platform tilt from error relative to its intended lean. Those distinctions change the interpretation of several videos.')
])
page('loop','One complete control interval','Implemented • the signal flow',[
 F('results/presentation_preparation_20261126/figures/current_loop.png','Blue arrows belong to the selected controller. The truth branch is evaluated separately and does not enter the observer-based command calculation.',210),
 B('At time t, read current simulated configuration and velocity plus the input applied over the preceding interval.',
 'Update the momentum residual. Locate support patches from its filtered force/moment at the current configuration.',
 'Update the filtered ground normal and load estimate. Build desired body attitude, joint-rate references and holding feedforward.',
 'Compute scheduled LQR feedback. Blend incoming-drive preparation and fade low-load torque. The optional heading helper is disabled in this selected baseline. Clip the final request.',
 'Hold that input for five 1 ms Gazebo/DART physics steps. Read the new state; repeat at 200 Hz on the simulated clock.'),
 C('The observer estimates; the LQR corrects; feedforward anticipates the motion implied by current geometry. These are distinct parts of the same closed loop.'),
 S('Selected settings: mode="hardware" is the repository name for the observer-and-locator interface; it is still used here in simulation. rolling_allocation="least_squares"; synchronize=True; tilt_reference=None; heading_gain=0. The generic OffroadParams default is a different contact-reading comparison mode.')
], '[C5] OffroadController.command; [C7] MomentumObserver; [C9] ContactSynchronizer; [C10] Gazebo loop/bridge.')
page('coordinates','Coordinates, axes and the five inputs','Dynamics • definitions',[
 P('The mechanism is a floating base (fork/platform), a carrier rotating about the fork’s y axis, and two shells rotating independently about the carrier’s x axis. Belt caps are attached to the carrier. Their surface motion is prescribed at contact rather than represented by additional dynamic belt joints.'),
 E(r'q=(x,y,z,Q_w,Q_x,Q_y,Q_z,\theta_s,\theta_A,\theta_B)'),
 E(r'v=(v_x,v_y,v_z,\omega_x,\omega_y,\omega_z,\dot\theta_s,\dot\theta_A,\dot\theta_B)'),
 T([['Object','Size / meaning'],['Configuration q','10 stored values: 3 position + 4 quaternion + 3 joint angles. The quaternion has one unit-norm constraint.'],['Generalized velocity v','9 independent velocity coordinates. Free-base angular velocity uses the body frame in this model; world force/moment conversion must respect that.'],['Local feedback error e','18 values: 9 configuration-tangent errors plus 9 velocity errors. Attitude error is a three-dimensional rotation error, not four quaternion differences.']],[125,350]),
 E(r'u=(\tau_s,\tau_A,\tau_B,s_A,s_B)'),
 T([['Channel','Units','Applied bound'],['Common shaft torque','N·m','±1.5'],['Each shell torque','N·m','±0.6'],['Each belt surface speed','m/s','±0.30']],[183,120,172]),
 S('Sphere radius R=0.12 m. Current total model mass is 1.86 kg. World frame: z is up. Motor torque, angular-rate target and belt surface speed must never be summed or compared without their declared scaling.')
], '[C1] config/model.yaml; [C2] model.py, sim.py; [T4] MuJoCo generalized-coordinate conventions.')
page('dynamics','The forward dynamics actually used','Dynamics • model and contact',[
 E(r'M(q)\dot v+c(q,v)=B_\tau\tau+f_p(q,v)+Q_{\mathrm{ext}}'),
 E(r'Q_{\mathrm{ext}}=\sum_i J_i(q)^T f_i'),
 P('<b>M</b> is the configuration-dependent inertia matrix. <b>c</b> contains gravity and velocity-dependent bias forces. <b>f<sub>p</sub></b> contains passive terms such as joint damping. The three joint torques enter through B<sub>τ</sub>. Contact forces f<sub>i</sub> are solved by the physics engine; they act through contact Jacobians J<sub>i</sub>.'),
 P('The recent work changes estimation, reference construction, routing and diagnostics; it does not introduce a new fundamental mass/bias equation. The generated multibody model and its standard dynamic terms remain the basis of the selected controller.'),
 H('Where do the belts appear?'),
 P('A belt command changes the target tangential velocity of its contacting surface. The resulting traction enters the solved contact force. It is not an extra generalized torque in B<sub>τ</sub>, and there are no two extra belt inertia states in this model.'),
 H('What determines contact force?'),
 B('Geometry and separation determine which surfaces can touch.',
 'Normal contact is unilateral: the ground can push on the wheel, but cannot pull it into the ground.',
 'Available tangential traction is friction-limited. A Coulomb cone is a useful conceptual description; the actual Gazebo/DART solver uses its configured discrete contact/friction formulation.',
 'Moving belt surfaces modify the tangential contact condition. If traction is insufficient, commanded surface motion need not become the desired body motion.'),
 C('The full dynamics are nonlinear and contact-dependent. The LQR is designed from local discrete approximations of this nonlinear step; the Gazebo robot does not evolve according to one fixed A,B matrix.'),
 S('Gazebo/DART produces the saved trajectories. MuJoCo provides the companion model, local design computations and observer model terms. Solver contact truth is logged for evaluation, separately from the selected controller input.')
], '[C2] model.py/sim.py; [C10] gazebo/plant.py and bridge; [T4] MuJoCo computation; [T6] Gazebo physics documentation.')
page('rolling','From body velocity to rolling references','Feedforward • kinematics',[
 P('A contact point moves because the base translates, the base rotates, the carrier/shell rotates, and possibly the belt surface moves. Ideal rolling asks for zero tangential surface velocity relative to the ground.'),
 E(r'P_t\left(v_c+\omega_W\times r+\sum_j\dot\theta_j(a_j\times r)+s\,d\right)=0'),
 S('r: center-to-contact vector; a<sub>j</sub>: world joint-axis direction; d: world belt travel direction; P<sub>t</sub>=I−nnᵀ: projection onto a local tangent plane. This is a physical explanation, not a claim that the native allocator solves this full equation.'),
 H('The native allocator’s actual approximation'),
 E(r'J_{xy}(q,\hat{r},\widehat{\mathrm{patch}})\,z\simeq- v_{\mathrm{ref},xy}'),
 E(r'z=(\dot\theta_s^*,\dot\theta_A^*,\dot\theta_B^*,s_A^*,s_B^*)'),
 P('The implemented rolling allocator stacks <b>world-horizontal XY</b> contact-Jacobian rows. It computes a minimum normalized-rate least-squares solution. Native rate scales are 5 rad/s for joints and 0.3 m/s for belts: these are normalization preferences, not hard angular-speed limits.'),
 B('The first three allocated values become joint-velocity references. The LQR supplies the torque needed to approach them.',
 'The two belt references also enter directly as rolling feedforward speed commands.',
 'Native least squares uses inferred contact identities/points, but does not numerically weight its rows by load shares. The experimental bounded allocator has a different weighted formulation.'),
 C('This is a velocity-consistency calculation. It does not prove the allocated motion can be accelerated, balanced or sustained within torque and friction limits.')
], '[C6] experiments.py:_rolling_allocation and rolling_feedforward_records; [C5] command reference construction.')
page('references_loop','How balance and holding references are built','Feedforward • not just trajectory tracking',[
 P('The desired body position comes from the timed path. The desired attitude also responds to the inferred ground normal and requested horizontal acceleration. A balanced platform on a slope may therefore be intentionally tilted relative to vertical.'),
 E(r'\ell_{xy}=\frac{a_{\mathrm{ref},xy}}{g}-\kappa\hat{n}_{xy},\qquad \kappa=\frac{m_{\mathrm{total}}R}{m_{\mathrm{upper}}h_{\mathrm{upper}}}'),
 P('Here ℓ is the horizontal part of the desired up-vector before its magnitude is bounded. The code converts that tilt into a quaternion and combines it with current yaw. The 28° setting caps the requested lean; it is not a proof of allowable terrain slope or actual tilt.'),
 H('Static holding feedforward'),
 E(r'\tau_{s,\mathrm{hold}}=mgR\,[a_s\times\hat{n}]_z'),
 E(r'\tau_{A,\mathrm{hold}}=w_A mgR\,[a_h\times\hat{n}]_z,\quad \tau_{B,\mathrm{hold}}=w_B mgR\,[a_h\times\hat{n}]_z'),
 P('The shell shares w<sub>A</sub>,w<sub>B</sub> route holding torque to inferred shell support. This component does not provide a belt holding torque: belt entries are zero because those channels are surface speeds.'),
 H('What feedback is intentionally relaxed?'),
 B('The target z is set to the current wheel-center z, suppressing vertical position error rather than pulling the wheel back to one global height. Vertical reference velocity is made consistent with the local plane.',
 'Absolute shell material-angle errors are suppressed. The shaft angle error is tied to rolling position error rather than blindly tracking accumulated angle.',
 'Position and linear-velocity errors are rotated into the current heading frame. Default yaw is free. Yaw-rate feedback can still act through LQR; the separate heading helper is disabled.'),
 S('A changing lean reference with an insufficient or zero desired angular-rate reference can create substantial attitude/rate demand. That was investigated in I-43; smoothing it improved some metrics but failed the broader comparison.')
], '[C5] lean_vector, reference_attitude, support_torque, _terrain_feedback; [E43].')
page('lqr','What LQR optimizes and what K means','Control • local feedback design',[
 P('The local 5 ms nonlinear step is differentiated numerically around a balanced operating point. Perturbing each error coordinate and each input gives the discrete matrices A and B.'),
 E(r'e_{k+1}\simeq A e_k+B\,\delta u_k'),
 E(r'\mathcal{J}=\sum_{k=0}^{\infty}\left(\tilde{e}_k^T Q\tilde{e}_k+\tilde{u}_k^T R_u\tilde{u}_k\right)'),
 P('State and input scales produce dimensionless errors and inputs before the Riccati equation is solved. Input bounds are used as scales in this design. <b>Scaling by a bound is not imposing an inequality.</b> The implementation reduces the local system to its reachable subspace and checks discarded unstable modes.'),
 E(r'K=(R_u+B^TPB)^{-1}B^TPA,\qquad \delta u=-Ke'),
 S('The formula is shown in one consistent coordinate system; the code transforms the normalized/reduced solution back to physical input and error units. It uses SciPy’s discrete algebraic Riccati solver.'),
 H('Gain scheduling'),
 P('A precomputed bank supplies local gains at different phases/supports. The scheduled gains are blended using inferred support loads; the selected shaft feedback row is kept from the ordinary local design. Default phase scheduling refers to world vertical, while the inferred normal still affects lean and holding torque.'),
 C('LQR gives an optimal policy for its declared local, unconstrained quadratic problem. That statement does not certify the full nonlinear, switching, clipped controller globally. We have not established such a certificate.'),
 S('Normalized effort weights are R<sub>u</sub>=diag(0.3,0.3,0.3,1,1). These weigh the five normalized command channels; they are not electrical power or a measured energy model. Configuration errors live in an 18-dimensional tangent/velocity representation.')
], '[C3] control.py:linearize/design_lqr; [C4] recovery_control.py:172,272 and offroad_control.py:303–341; [C5] _terrain_feedback; theoretical background [T1,T5].')
page('constraints','Why an effort penalty is not a hard constraint','Investigation 1 • the central distinction',[
 C('<b>Your concern is correct:</b> command feasibility is a controller-design issue. But the existing clamp is working. The missing feature is anticipation of saturation and its effect on balance/tracking, not the existence of a final numeric limit.'),
 H('A deliberately simple quadratic example'),
 E(r'\min_u\ (x+u)^2+r_u u^2\quad\Longrightarrow\quad u^*=-\frac{x}{1+r_u}'),
 P('For x=10 and r<sub>u</sub>=1, the unconstrained optimum is u=−5. If the allowed range is [−1,1], the optimum still lies outside it because the optimization never included that range. Adding |u|≤1 changes the problem; clipping afterward changes the action but not the prediction that generated it. This is a one-step teaching example, not a robot test.'),
 H('Three different “effort/limit” mechanisms in our code'),
 T([['Mechanism','What it really does'],['LQR input cost','Soft cost on normalized input deviation. It can prefer a large command when predicted state error is costly.'],['Rolling-rate allocation','Native least squares chooses normalized drive rates. Experimental bounded/feasible modes have speed/rate inequalities; those modes are inactive in the selected baseline.'],['Final clip','Hard-bounds the total issued command after feedback, feedforward and preparation. It guarantees those numeric input bounds, not successful balance or tracking.']],[146,329]),
 P('Increasing the LQR effort penalty can reduce demand, but generally also weakens correction. A nonzero linear gain can produce an arbitrarily large request for sufficiently large error. A finite penalty therefore cannot enforce a fixed bound for every error.'),
 S('Native joint-rate and acceleration reference limits are not enforced merely because their settings exist in OffroadParams. Only the active branch applies them. Constraints on references also do not constrain every later feedback addition.')
], '[C3] normalized Q/R design; [C5] branch selection and final np.clip; [C6] native allocation; [T1] constrained versus unconstrained LQR.')
page('composition','Where the command changes before application','Control • exact implementation order',[
 E(r'u_0=u_{\mathrm{eq}}+u_{\mathrm{roll}}+u_{\mathrm{hold}}-K(\phi,\hat{w})e'),
 E(r'u_1=(1-\beta)\odot u_0+\beta\odot u_{\mathrm{prepare}}'),
 E(r'u_2=u_1+u_{\mathrm{heading}},\qquad u_3=\mathrm{fade}(u_2)'),
 E(r'u_{\mathrm{applied},i}=\min(b_i,\max(-b_i,u_{3,i}))'),
 P('The optional heading term is zero in the selected baseline. The request in a saved “requested” field is the result before final clipping, after the active preparation/heading/fade additions. The shadow routing audit also saves an earlier <b>pre-synchronization</b> command so one change can be isolated. Do not compare these as though they were the same signal.'),
 H('Why clipping can change the intended behavior'),
 B('Feedback is designed around an approximate relationship between requested input and the next state. Saturation changes that relationship.',
 'One drive may hit its bound before another. Independent clipping can change the direction and relative balance/travel contributions of the combined correction.',
 'References may continue moving while the robot cannot keep up, increasing state error and requesting still more correction. That is not automatically an integral-windup claim; the feedback includes evolving references/history without a generic integral state.'),
 C('A QP that only minimizes distance to u<sub>request</sub> under independent box bounds can be equivalent to clipping. A meaningful improvement needs dynamic/geometry-aware priorities or feasible references, rather than renaming the same clamp “optimization”.'),
 S('A feasible controller still cannot guarantee an impossible task. When balance, path motion and bounds conflict, the design must explicitly decide what may relax and when to stop or recover.')
], '[C5] OffroadController.command and _terrain_feedback; [C2] Plant.apply_input repeats the input bounds.')
page('saturation','The saturated event, with numbers','Investigation 1 • observed saved trace',[
 P('At t=6.825 s in the original Mars figure-eight, the controller correctly selects belt A. Desired horizontal body velocity is approximately (0.03244,0.03128) m/s, a magnitude of 0.0451 m/s. Nevertheless, belt A requests 0.83474 m/s.'),
 T([['Belt-A contribution','m/s'],['Attitude-error feedback','+0.54555'],['Angular-velocity-error feedback','+0.20210'],['Linear-velocity-error feedback','+0.07474'],['Position-error feedback','+0.01918'],['Joint-velocity-error feedback','+0.00224'],['Total LQR feedback','+0.84380'],['Rolling feedforward','−0.00906'],['Total requested / applied','+0.83474 / +0.30000']],[330,145]),
 C('Attitude and angular-rate terms account for 88.6% of this LQR contribution. This particular large request is not primarily the rolling feedforward trying to execute a fast path.'),
 P('Actual absolute tilt is about 7.64° and the reference tilt about 12.03°. The full up-vector error is about 5.29°; subtracting scalar tilt magnitudes is not the same geometric error. The large balance correction occurs before the sustained path departure at 6.860 s.'),
 S('Holding, preparation, heading, fade and equilibrium contributions to this belt channel are zero at this sample. The exact sums were rechecked from saved gain/error/full-precision arrays. This is one diagnostic sample within a logged sequence, not proof of the entire physical cause.')
], '[E42] shared feedback-term audit; [E46] full-precision historical replay; new read-only controller audit [A1].')
page('observer','The momentum observer, step by step','Estimation • model residual, not a force sensor',[
 P('The observer asks: how much of the observed momentum change cannot be explained by the model and the input we just applied? That remainder estimates external generalized force. In the present tests, ground reaction is the main external contribution; a push or model error can also appear in the residual.'),
 E(r'p=M(q)v,\qquad \dot p=B_\tau\tau+f_p-c+\dot Mv+Q_{\mathrm{ext}}'),
 H('The implemented discrete calculation'),
 P('At time k, use current state, the previous state, and the <b>input held during the completed interval</b>. Evaluate M, bias and passive force at both endpoints. Approximate their impulse with the trapezoidal rule and include the changing-mass-matrix term:'),
 E(r'I_k=\Delta t\left[B_\tau\tau_{k-1}+\frac{f_{p,k-1}+f_{p,k}}{2}-\frac{c_{k-1}+c_k}{2}\right]+(M_k-M_{k-1})\frac{v_{k-1}+v_k}{2}',.68),
 E(r'\bar{Q}_{\mathrm{ext},k}=\frac{M_kv_k-M_{k-1}v_{k-1}-I_k}{\Delta t}'),
 P('The known three motor torques are reconstructed with their modeled gains/limits. The belt surface-speed channels affect traction through contact, not through two fictitious generalized torque inputs. No direct acceleration measurement or solved contact list is read.'),
 C('“No acceleration sensor” does not mean noise or model error disappears. This implementation differences estimated momentum over a finite interval, then filters it. It is a discrete momentum-residual adaptation, not a verbatim implementation of every continuous observer equation in the reference paper.'),
 S('This implementation is related to generalized-momentum disturbance/residual observers. Credit De Luca and Mattone’s 2003 generalized-momenta work and 2005 collision-detection paper for the underlying idea; the floating-base discrete calculation and support interpretation are the project implementation.')
], '[C7] contact_sensing.py:227–356; exact derivation and frames [A2]. Background only [T2,T3]; project equations are traced to code.')
page('observer_time','The observer’s output has a time and a frame','Estimation • three clocks, several filters',[
 P('For the six wrench rows, apply the following filter <b>after</b> rotating moment into the common world frame. The interval residual below denotes that frame-aligned quantity.'),
 E(r'\hat{Q}_k=\hat{Q}_{k-1}+\alpha(\bar{Q}_{\mathrm{ext},k}-\hat{Q}_{k-1}),\qquad \alpha=1-e^{-100\Delta t}'),
 T([['Signal / operation','Meaning in the selected loop'],['Previous applied input','The command held from t−5 ms to t. Using the newly calculated input here would be a timing error.'],['Interval residual','An inferred average over the completed 5 ms interval, not instantaneous truth at t.'],['Filtered wrench','First-order memory of residuals. At Δt=5 ms, α≈0.39347 and nominal time constant=10 ms.'],['Normal / rim filtering','Separate selected normal filter:20 ms; rim treatment:200 ms. These are not the filter that directly supplies patch shares.'],['New applied command','Computed at t, then held over the next 5 ms.']],[133,342]),
 H('Frame bookkeeping'),
 P('Base translation/linear velocity use world coordinates. The mirror’s free-base angular velocity uses body coordinates; Gazebo’s world angular velocity is rotated into that frame. Residual moment rows are rotated to world coordinates with a sign-consistent midpoint quaternion for the completed interval. The output is a world force F and moment m about the wheel center.'),
 C('A 10 ms time constant is not a fixed 10 ms delay. At a transition, filtered force history and current geometry can combine to produce early, late or chattering patch labels. Advancing every estimate by a fixed time is not justified by that time constant.'),
 S('This is 200 Hz on the simulated control clock. The saved studies did not establish a per-command real-time computation deadline.')
], '[C7] _to_world/update/wrench; [C10] GazeboPlant._pull; [A2] timing/frame audit.')
page('contact','From force and moment to a contact normal','Contact estimation • the geometric assumption',[
 P('For one ideal spherical contact, let c be the wheel center, R its radius, n the outward ground normal, and r=−Rn the center-to-contact vector. The moment about the center obeys:'),
 E(r'm=r\times F=-R\,n\times F,\qquad p_c=c-Rn'),
 P('The force and moment determine the component of n perpendicular to F. Unit length and a compressive force select the remaining component:'),
 E(r'n_\perp=-\frac{F\times m}{R\|F\|^2}'),
 E(r'n=\sqrt{1-\|n_\perp\|^2}\,\frac{F}{\|F\|}+n_\perp'),
 H('What the code does at the limits'),
 B('With almost zero force, the normal is undefined and the locator returns no support.',
 'If the inferred perpendicular component is too large for a unit normal, the code normalizes that component. This keeps the output bounded; it does not prove the wrench is physically consistent with one spherical point.',
 'Several real contacts can create one equivalent resultant. Their individual forces, identities and positions are generally not uniquely recoverable from that resultant.'),
 C('The observer does not directly estimate “belt A.” It first estimates a net wrench. Geometry then turns that wrench into an equivalent normal/point, and a separate classifier assigns the patch. Each step can add a different kind of error.'),
 S('The single-point relation assumes no independent contact couple about the point. The displayed equations explain intrinsic_normal exactly in its regular case; rim and distributed contact are handled by additional model assumptions, not by proving this single-point model universally exact.')
], '[C8] contact_sensing.py:201–217,379–465; numerical/source audit [A2].')
page('locator','Which shell or belt gets the load?','Contact estimation • classification and use',[
 P('The known sphere is divided into shell bands, polar belt caps and the equatorial gap. For a current patch-axis a(q), sign s and center-to-support vector r, the locator checks a signed axial coordinate:'),
 E(r'\xi=s\,a(q)^Tr,\qquad \xi_{\min}\leq\xi\leq\xi_{\max}'),
 T([['Patch family','Ideal signed axial band'],['Shell A or B','10.0000 to115.6547 mm'],['Belt A or B','115.6547 to120.0000 mm']],[175,300]),
 P('A first matching patch receives the inferred load. In the equatorial gap, two known rim points are constructed and a clipped lever rule divides their shares. The selected locator requires compressive load above 5% of model weight before returning records.'),
 H('Those records have several downstream jobs'),
 B('Patch/point/Jacobian: determine rolling rate references and belt feedforward.',
 'Inferred load shares: blend LQR gains and route shell holding torque.',
 'Filtered normal: changes lean and holding references; it is distinct from the raw support-share classification.',
 'Short dropouts: the previous loaded gain/load and rolling reference can be held rather than switching abruptly to an unrelated mode.'),
 P('Slip is estimated from velocity consistency at the inferred point. The contact estimator also maintains a friction estimate. In this selected controller, that friction estimate is diagnostic; it does not impose a hard friction-cone constraint on the final command.'),
 C('A small point error need not imply correct drive assignment. If the point is only tens of micrometers from a band boundary, an equally small change can select a different input channel. Shared-rim handling is not a general arbitrary-multi-contact estimator.')
], '[C8] SupportLocator and ContactEstimator; [C5] _terrain_feedback/_fade_when_unloaded; [C6] rolling branches; [A2].')
page('timing','Was the robot moving during calculation?','Investigation 3 • test the latency explanation',[
 C('<b>The saved tests do not support that mechanism.</b> Gazebo advances in finite blocking batches after the controller returns its command. Simulated time does not continue advancing during Python calculation between those batches.'),
 E(r'(q_k,v_k,u_{k-1})\ \longrightarrow\ \hat F_k,\hat m_k\ \longrightarrow\ u_k\ \longrightarrow\ 5\ \mathrm{physics\ steps}'),
 P('A long calculation can make the experiment slower in wall-clock time. In this lockstep execution it does not automatically mean the simulated robot has moved ahead before that command is applied. This is different from claiming computation takes zero time.'),
 H('What the recordings do and do not contain'),
 T([['Available saved evidence','Not recorded in these audited traces'],['Simulation times and state arrays','Per-command CPU start/finish and deadlines'],['Prior/current requests and applied inputs','Command transport/send/receipt latency'],['Observer wrench, patch/load records','An independently measured real-time delay model'],['Truth contact records; I-46 raw1ms substeps','Workspace inventory found no ROS .bag/.db3/.mcap recordings']],[237,238]),
 P('We inspected NPZ state/command arrays and JSON/CSV/log metadata rather than assuming ROS bags exist. Aggregate run wall time cannot isolate a delayed control sample.'),
 H('What can still produce a discrepancy?'),
 B('The inferred wrench summarizes a completed interval and is filtered.',
 'The locator evaluates that historical information at the current configuration.',
 'Last-solver contact positions and integrated state use distinct sampling conventions.',
 'Ideal spherical patch bands, finite collision geometry and contact tolerances differ near a boundary.'),
 S('An estimate need not always be delayed: an estimator can be designed to estimate a current state, and a predictor can extrapolate a future state. Here the specific wrench signal is history-bearing; its patch-label error is not established as one fixed lag.')
], '[C10] offroad_experiments.py:248; gazebo/plant.py:173; bridge.py:82. Read-only clock/artifact inventory [A2].')
page('contact_event','Close point, different drive: inspect the boundary','Investigation 3 • observed historical samples',[
 T([['Time s','Last-solver patch','Filtered route','Interval route','Filtered point gap'],['6.725','Shell A','Belt A','Shell A','0.462 mm'],['6.825','Belt A','Belt A','Belt A','0.139 mm'],['6.855','Belt A','Belt A','Shell A','0.193 mm'],['6.860','Belt A','Shell A','Shell A','0.183 mm'],['6.865','Shell A','Shell A','Belt A','0.331 mm']],[55,101,103,103,113]),
 P('At 6.855 s the interval-mean point is <b>closer</b> to the saved equivalent point (0.087 mm versus0.193 mm), but its patch label disagrees while the filtered label agrees. “Faster” and “closer” therefore do not automatically imply correct actuation routing.'),
 C('At6.860 s, the filtered estimate is only0.183 mm away, but its selected shell upper-band margin is just0.0259 mm. That is about26 micrometers: much smaller than the point discrepancy.'),
 H('The necessary qualification'),
 P('At the current integrated pose, the saved true equivalent belt point itself projects about0.0782 mm outside the ideal belt band. At6.855 s it is0.0140 mm outside. Last-solver force/point, integrated pose and ideal locator geometry are not identical objects.'),
 P('This confirms <b>support-information disagreement</b>. It does not isolate instantaneous geometric misclassification, computational latency or filter lag as the unique cause. No post-hoc clock alignment was used to eliminate the disagreement.'),
 P('At6.825 s, support agrees and the belt is already over-requested. The I-42 diagnostic true-share intervention changes feedback routing but leaves the first6.860 s departure unchanged. That narrows the explanation without proving contact uncertainty is harmless.'),
 S('All listed values are re-read from identical saved historical states, not newly simulated tests. “Point gap” refers to the saved load-weighted equivalent contact point and is not a claim of exact physical contact resolution.')
], '[A2] historical_samples_6p7_to_6p9.json and current-pose band projections; [E42,E46].')
page('timing_results','What the faster-signal comparison showed','Investigation 3 • observed, shadow only',[
 T([['Fresh common transfer','Filtered offset','Interval offset'],['Mars shell→belt at3.050s','0ms','0ms'],['Mars belt→shell at3.870s','−10ms','−5ms'],['Mars shell→belt at11.585s','−5ms','−5ms'],['Flat transfers (four)','−5ms each','−5ms each']],[253,111,111]),
 S('Seven transfers from two16s runs;5ms control grid. Stable family share≥0.8 must span at least15ms.'),
 P('Negative means <b>early</b> relative to preceding-interval mean contact truth. Across these seven common stable transfers, median absolute timing improvement is0 ms. These data contradict an “always late by the computation time” description.'),
 T([['Routing mismatch-equivalent duration','Mars filtered→interval','Flat filtered→interval'],['Against preceding5ms mean','5→0ms','10→10ms'],['Against following held mean, auxiliary','5→5ms','0→0ms']],[237,119,119]),
 P('The second row uses future contact only in an evaluator lookup along the actual native history. It is not information supplied to the controller, and it is not an executed alternative trajectory. A preceding-mean scoring benefit is not automatically improved routing for the next command hold.'),
 H('Original figure-eight, scored separately'),
 P('Historical last-1-ms disagreement improves90→55 ms, but the first useful route correction is at6.725 s, after the first native belt request at its bound at6.700 s. The maximum requested command jump stays unchanged. The historical and fresh truth clocks are not pooled.'),
 C('The registered screen failed, so the filtered observer remains selected. This is evidence against this particular fast-routing change, not proof that every contact handoff is reliable or that filtering never matters.'),
 S('The alternative changes only same-state feedback-gain/holding shares in shadow. Rolling feedforward, normal, references, preparation and actual applied commands remain native. No closed-loop driving improvement was tested for that alternative.')
], '[E46] registered report and paired_events.csv; [A2] seven_fresh_transitions.csv and clock-qualified audit.')
page('preview','What prediction is actually active?','Preview • geometric preparation, not future forces',[
 P('The selected loop does use a small prediction: a 120 ms constant-velocity geometric preview. It advances a private configuration copy using current velocity and checks how shell/belt bands approach the estimated local plane.'),
 E(r'q_{\mathrm{preview}}(t+h)=\operatorname{IntegratePos}(q(t),v(t)h),\qquad 0\leq h\leq0.12\ \mathrm{s}'),
 B('For an approaching unloaded belt, request an incoming surface speed that is closer to the tangential speed at expected contact.',
 'For an unloaded shell, form a target angular speed and bounded preparation torque. Its reaction can still affect the carrier/platform.',
 'Preparation blend is zero for a drive carrying at least5% of current inferred normal load. Unloaded shell blend is capped at0.35.',
 'The shell target-speed ceiling is20 rad/s. It is a preparation reference ceiling, not a general hard bound on actual shell speed.'),
 H('What it does not compute'),
 P('It does not forecast the exact future normal force, load transfer, slip, impact or changing terrain. It does not integrate the complete future closed-loop contact dynamics. It assumes the current velocity and estimated plane are useful for short-range geometric preparation.'),
 C('Our present contact estimate is reactive; incoming-drive preparation is predictive in a limited geometric sense. Saying “we have no prediction” is too strong, and saying “we predict future contact forces” is also too strong.'),
 S('Longer dynamic contact forecasts were evaluated as separate diagnostics and are not feeding selected live commands. The camera branch is outside this guide’s scope. The selected controller still needs current observer-based support for feedback.')
], '[C9] synchronization.py:ContactSynchronizer.preview; [C5] blending and held plane; [A2].')
page('authority','Why two drives can point the same way','Investigation 2 • the geometric cause',[
 P('A rotating joint moves a contact point along a<sub>j</sub>×r. Horizontal body motion needed for rolling is opposite that contact-surface velocity. When the shaft and loaded shell axes become geometrically aligned in their effect at contact, their two horizontal velocity vectors become parallel.'),
 E(r'v_{xy}=D(q,r)\nu,\qquad D_j=-[a_j\times r]_{xy},\quad \nu=(\dot\theta_s,\dot\theta_h)'),
 F('results/presentation_preparation_20261126/motion_audit/figures/drive_direction_example.png','Verified analytic reconstruction of a saved single-shell-B example. Same platform attitude, different shaft phase. Arrow columns describe motion per joint angular rate, not torque directions.',218),
 C('This is possible with ONE loaded hemisphere. It is not a consequence of “both hemispheres touching.” Tilt matters, but shaft phase, contact direction and requested travel direction also matter.'),
 P('At roll−20°, pitch/yaw0°, flat normal and shaft phase−90°, both drive columns point along x. The robot can still move along x, but these two instantaneous rolling channels provide no independent y motion.'),
 S('Yaw on level ground rotates the weak direction relative to a world-frame request; a pure yaw rotation does not by itself change singular values. Slope-normal and configuration changes can change the geometry itself. Instantaneous rolling authority is not a full dynamic controllability or balance certificate.')
], '[E12] saved same_tilt_example.json; [A3] independent NumPy-only reconstruction and figure. No grid rescan or physics run.')
page('authority_math','The same attitude can have different authority','Investigation 2 • a numerical example',[
 P('For the single-shell-B example, the two columns below map joint angular rates [shaft,shell B] to horizontal body rolling velocity. Entries have units meters per radian.'),
 dict(type='code',text='Phase -34.377 deg: D = [[0.112763,  0.023174],\n                        [0.000000, -0.099040]]  m/rad'),
 dict(type='code',text='Phase -90.000 deg: D = [[0.112763, 0.041042],\n                        [0.000000, 0.000000]]  m/rad'),
 P('The matrix equations use the same roll−20° and ground normal. Only shaft phase changes. The physical singular values at−90° are approximately[0.12,0] m/rad. At−34.377°, multiplying columns by the declared5 rad/s normalization gives singular values[0.600,0.4653] m/s.'),
 T([['Requested body velocity at−90°','Result'],['(0.060,0) m/s','Achievable kinematically with rates about(0.46985,0.17101) rad/s.'],['(0,0.060) m/s','No solution using these two instantaneous rolling channels. Both columns have zero y component.']],[214,261]),
 H('Near alignment, rather than perfect alignment'),
 E(r'D=U\Sigma V^T,\qquad \nu^*=V\Sigma^{+}U^T v_{\mathrm{ref}}'),
 P('A small singular value means a weak motion direction. Dividing a requested component by that small number produces a large rate demand, often with opposing drive contributions. At exactly zero, the requested component is outside the matrix’s range; a pseudoinverse cannot create it.'),
 C('The correct response can include accepting motion error in the weak direction. But the body feedback reference must agree with that relaxation, and enough actuation must remain available for balance. Tilt-only weighting cannot capture all of this.')
], '[A3] saved-example analytic reconstruction, verified against [E12]. This is a kinematic explanation, not a new dynamic test.')
page('authority_event','A saved near-singular event: effort versus motion','Investigation 2 • rate demand is not torque demand',[
 P('A separate original diagnostic figure-eight run has a near-singular sample at31.650 s. Reconstructing its saved shell-B contact/configuration gives the matrix below; this is not the6.825 s balance event.'),
 dict(type='code',text='D = [[0.00444446, 0.00120139],\n     [0.11536132, 0.03199865]]  m/rad; condition number ≈3961'),
 T([['Same-state allocation calculation','Shaft / shell-B references','Velocity residual'],['Reconstructed undamped least squares','+502.66 /−1811.23 rad/s','Approximately matches requested velocity algebraically'],['Damped, λ=0.01 m/s cost scale','+0.3833 /−0.3396 rad/s','56.84 mm/s remaining mismatch']],[188,154,133]),
 P('Here S=diag(5,5) rad/s and v<sub>ref</sub>=(0.05808847,0.03117049) m/s. These are reconstructions from stored float32 pose/point values. The actual logged rates are +488.60/−1760.52 rad/s; near-singularity amplifies small reconstruction differences. The damped result is algebra only, never actuated. Rates are references, not torques or attained speeds.'),
 H('The regularized objective explains your proposed tradeoff'),
 E(r'\min_z\ \|D S z-v_{\mathrm{ref}}\|^2+\lambda^2\|z\|^2,\qquad \nu=S z'),
 P('A positive effort weight accepts velocity mismatch instead of huge opposing rates. Bounded formulations can additionally constrain the reference. However, if LQR still tracks the original body velocity/position aggressively, it can demand large corrections downstream.'),
 C('There are two distinct feasibility problems: kinematic allocation of drive rates and dynamic feasibility of balance/travel inputs. A successful fix for one is not automatically a fix for the other.'),
 S('Source run: original diagnostic baseline, plain_figure8, phase+0.90, yaw45°,44 s, Gazebo/DART Mars2021 DEM. The minimum-norm historical implementation predates the later allocation-mode enum. No trajectory was rerun.')
], '[A3] early_saved_event and exact run metadata; saved original NPZ index6330; [E12] direction/continuity study.')
page('turns','What “overshoot” means in our logs','Investigation 4 • separate three measurements',[
 T([['Metric','Question it answers'],['Timed position error','At this exact time, how far is the robot from the timed reference?'],['Local geometric path distance','How far is the robot from the nearby reference curve, allowing limited progress mismatch?'],['Geometric overshoot / reversal lag','Did a selected coordinate pass its reference extremum? When did its direction reverse?']],[160,315]),
 F('results/presentation_preparation_20261126/motion_audit/figures/turn_geometry_and_timing.png','Replotted from the six saved I-41 turn events. A component reversal is not necessarily a stop of the whole robot.',235),
 P('At the first Mars turn, timed error is86.46 mm, local path distance11.52 mm, geometric overshoot0 mm and reversal lag+1.101 s. The robot turns late but does not pass that coordinate’s extremum. Calling all of this “overshoot” would hide the actual behavior.'),
 P('The corresponding flat-ground first turn has8.03 mm geometric overshoot and+0.193 s reversal lag. Turning error can therefore occur without the Mars bump. This does not isolate one controller term, because the terrain cases have different evolving states and support histories.'),
 S('Overshoot is measured in declared[−1,+2] s turn windows. A100 ms dwell qualifies detected reversal time, not the overshoot magnitude. Late turns and overshoot can coexist, but their magnitudes should be scored separately.')
], '[E41] event/turn definitions; [A3] exact saved turn tables and plotting audit.')
page('tradeoffs','Why earlier “improvements” were not adopted','Iteration history • useful negative results',[
 T([['Saved trial','Useful observation','Reason it was not a general fix'],['Uniform rolling damping','Reduced one case’s largest joint reference1760.5→4.4 rad/s.','Another phase RMS51.1→86.4 mm; weakened useful motion.'],['Bounded references / effort weights','Reference limits held; several route metrics improved.','Feedback/actual speed not guaranteed; phase and route regressions remained.'],['Feasible body-reference variants','Accepted velocity and relaxed errors were made more consistent.','Other routes/goal errors regressed; some relaxed-reference scores hid original-goal error.'],['I-42 true feedback shares','Corrected selected routing differences.','All four arms still had the first departure at6.860 s.'],['I-42 preparation masked','Later moving RMS50.486→41.681 mm in that run.','Early RMS gain only0.29%; four of six later overshoots worsened.'],['I-43 lean + consistent rate','Early RMS38.661→29.893 mm; belt demand0.835→0.491 m/s.','Full raw-target peak lean8.709→13.581°; four of six turns worsened.'],['I-46 faster wrench routing','Some historical label disagreement reduced.','Seven fresh paired events had0 ms median timing benefit; largest command jumps unchanged.']],[115,175,185]),
 C('The experiments show coupling and tradeoffs, not that constraints or better estimation are bad ideas. A candidate must satisfy balance, original-path tracking, turning and completion criteria together.'),
 S('Rows refer to separately declared saved campaigns and their own baselines. Do not combine them into one statistical trial count or attribute all changes to one scalar cost. No new candidate was actuated for this PDF.')
], '[A1] saved candidate table with original report paths; [E12,E42,E43,E46].')
page('turn_tradeoffs','Preparation and lean shaping change later turns','Iteration history • evidence against one-metric tuning',[
 F('results/presentation_preparation_20261126/motion_audit/figures/intervention_turn_tradeoffs.png','Saved six-turn comparisons. The interventions can improve one metric while worsening another; they do not support permanent preparation shutdown or automatic adoption of the lean shaper.',260),
 P('For example, masking preparation reduced first-turn reversal lag from1.101 to0.832 s, but introduced18.05 mm overshoot where the baseline had0. At the final listed turn, geometric overshoot increased60.20→68.41 mm. These later events occurred after the preparation intervention window ended; changed physical/controller history persisted.'),
 P('Adding consistent desired tilt rate improved raw-reference lean in the demand window but increased peak belt demand relative to the angle-only arm (0.376→0.491 m/s). The arms used the same generator settings, while their closed-loop angle histories differ. The complete candidate still failed the declared raw-target attitude and turn screen. A smaller error against a newly softened target is not enough to demonstrate improved original behavior.'),
 H('What is known about the cause?'),
 B('Confirmed: response is delayed at some reversals; real geometric overshoot occurs at others; problems appear on flat as well as Mars cases.',
 'Confirmed: reference, balance feedback, preparation reactions and carried history affect the outcome.',
 'Not isolated: a unique term or one tuning value that explains and fixes all turns. Saturation and weak rolling directions can contribute, but the saved turn evidence alone does not prove they cause each overshoot.'),
 S('The current LQR baseline was retained. We should explain that decision as a result of measured nonregression criteria, not as a claim that every trial failed in every way.')
], '[E42] turn addendum; [E43] six-turn result; [A3] saved-metric reconstruction.')
page('next','What the investigation supports next','Proposed • no implementation in this task',[
 C('Prioritize constraint-consistent reference/control design and measured turn response. Do not assume a faster contact filter or a larger effort weight is the missing universal fix.'),
 H('1. Keep identifying which part hits the limit'),
 P('For each saved correction, align body/reference attitude and rate, per-term LQR contributions, rolling/holding/preparation additions, requested versus applied input, contact margins and turning metrics. The6.825 s and31.650 s examples must remain distinct mechanisms.'),
 H('2. Make relaxation consistent across the loop'),
 P('If motion along a weak direction is intentionally reduced, update the body velocity/position feedback reference consistently. Check the <b>total</b> balance-plus-travel command, not only kinematic rate references. Retain meaningful balance priority; record infeasibility rather than promising every requested motion can be performed.'),
 H('3. A candidate must add something beyond clipping'),
 P('A reference governor or an explicitly constrained local model can be investigated while retaining LQR as the nominal feedback. A purely box-constrained projection of the requested input may reproduce today’s clipping. Dynamic/geometry-aware constraints or priorities are needed to change the feasibility decision meaningfully.'),
 H('4. Treat contact ambiguity as an uncertainty signal'),
 P('Near a patch edge, report margin and support disagreement. The current observer cannot uniquely recover arbitrary simultaneous contacts. Any future handoff rule should be checked against both early and late transitions, existing gain blending/preparation and the correct time convention.'),
 H('5. Require multiple objectives to pass together'),
 P('A later candidate should be compared on saturation, raw/reference-relative attitude, original-path error, turn overshoot/lag, settling and completion. Preserve regressions. The present task deliberately stops at saved-data diagnosis and the study guide.'),
 S('These are proposed design directions, not a stability proof, validated controller or an instruction to rerun simulations. The prior bounded/feasible/tilt trials remain relevant negative evidence that a simplistic repeat is insufficient.')
], '[A1–A3] combined source/log audit; [E12,E41–E43,E46].')
page('qa1','Practice: control and constraint questions','Professor Q&A • answer, then point to evidence',[
 H('Why does LQR exceed limits if we penalize effort?'),
 P('Because a finite cost discourages input; it does not impose |u|≤b. In this implementation the Riccati problem is unconstrained. The issued input is bounded later by clipping, which does not make the requested state transition feasible.'),
 H('So is the controller broken?'),
 P('The clipping mechanism is working as specified. The design is incomplete with respect to constrained closed-loop feasibility and balance/tracking priorities. We have identified that gap; we have not solved it or proved it is the only cause of every deviation.'),
 H('Why not just raise the effort weight?'),
 P('It can reduce input demand while increasing response lag or balance error. Earlier damping/weight trials show those tradeoffs. Also, a rate-allocation cost cannot directly fix a large LQR balance term; inspect the contribution first.'),
 H('Does bounded allocation constrain the complete controller?'),
 P('No. It constrains selected drive-rate references under its assumptions. The downstream feedback, holding and preparation contributions can still request more input. Actual joint speed is a dynamic state, not the same thing as a capped rate reference.'),
 H('Are you optimizing torque or speed?'),
 P('Both kinds of channels appear, but at different stages: actual inputs are three torques and two belt surface speeds; the rolling allocator computes three angular-rate references plus two surface-speed references. Their normalized costs are not motor power.'),
 H('Is the LQR globally stable?'),
 P('No such claim is established. Local design/eigenvalue checks do not certify nonlinear contact switches, gain interpolation, estimation errors, friction and saturation. Successful selected trials are empirical evidence, not a global proof.'),
 H('Why can small desired body speed require large belt speed?'),
 P('A local surface speed is not body translation. At6.825 s the body target is about0.045 m/s, but the LQR requests substantial belt motion to correct attitude and angular rate. At31.650 s a separate weak-geometry mechanism produces extreme joint-rate references.')
], 'Answers are supported by the saved decomposition and source/log audits [A1,A3], not an assumed new controller.')
page('qa2','Practice: estimation, prediction and evidence','Professor Q&A • use the correct clock',[
 H('Does the momentum observer read contact truth?'),
 P('Not in the selected observer-and-locator loop. It reads current/prior configuration and velocity, known prior applied commands and model terms. The separate evaluator reads solved contacts. Exact simulated body state is supplied; that assumption must be stated.'),
 H('Is the contact point old because the calculation takes time?'),
 P('The wrench is history-bearing, but simulated physics does not advance between finite lockstep runs. These logs do not measure CPU/transport delay as the cause. We should separately discuss filtering, completed-interval information and geometry at current pose.'),
 H('Why can a close contact point select the wrong motor?'),
 P('Identity is a discrete band decision. The point error can be larger than its tiny boundary margin. Also, last-solver truth and current integrated pose need not lie on exactly the same ideal band. The6.860 s sample demonstrates disagreement, not a uniquely isolated latency mechanism.'),
 H('Are we predicting contact?'),
 P('The support estimate is reactive. The active synchronizer predicts geometric exposure up to120 ms under constant velocity on the estimated plane. It prepares drives; it does not predict exact future forces or guarantee a handoff.'),
 H('Would perfect contact solve the tracking problem?'),
 P('Not demonstrated. The I-42 diagnostic true-share substitution changes routing without preventing the first departure. At an earlier inspected instant both assignments agree while the belt is already over-requested.'),
 H('Can the observer resolve several independent contacts?'),
 P('Not generally from one net wrench. The implementation returns an equivalent spherical support and has a specific two-rim lever model. Those assumptions do not identify every possible simultaneous contact distribution.'),
 H('What did the latest contact-filter test actually improve?'),
 P('Some historical labels and one fresh transition improved, but paired median timing benefit was0 ms and maximum command jumps were unchanged. The faster variant was shadow-only; we did not demonstrate a different driving trajectory.')
], 'Observer/frame/clock/source evidence [A2]; scope and registered decision [E46].')
page('talk','A presentation that fits within five minutes','Assignment preparation • use evidence, not every equation',[
 P('Aim for4:30 within the five-minute limit, including one short video excerpt. Keep the detailed derivations in reserve for questions.'),
 T([['Slide / time budget','What to explain','Evidence / credit'],['1. Problem and mechanism —30s','One wheel, changing shell/belt contact, simultaneous balance and travel. Configuration changes useful drive directions.','Mechanism sketch; credit supplied CAD/design.'],['2. Current approach —65s','State→momentum observer→support estimate→rolling/holding feedforward + scheduled LQR→preparation→bounds. Explain what is estimated.','Loop diagram; [T1–T6] plus code IDs.'],['3. Demonstrated behavior —65s','One compact Mars/flat example, with terrain, reference length and conditions. Show15–20s of the saved-state video.','I-46 native behavior: Mars RMS13.27mm/final0.396mm; not a filter improvement.'],['4. What failed and why —70s','Show0.835 request versus0.3 applied and the mostly-balance decomposition. Briefly show parallel drive arrows. Contact lag alone was insufficient.','I-42/E46 saved traces; geometry example.'],['5. Iteration and next question —40s','Costs/constraints and contact changes helped selected metrics but regressed others. Retain baseline; investigate consistent feasibility and turns.','One result/tradeoff table; proposed work labeled.']],[146,205,124]),
 H('A concise opening'),
 P('“We study a spherical-wheel robot with changing drive authority. We balance and track motion in simulation while estimating support from dynamics instead of reading contact truth.”'),
 H('A concise conclusion'),
 P('“Our diagnostics distinguish actuator saturation, weak drive geometry and contact disagreement. Simple changes did not consistently improve all metrics. We retained the baseline and identified constrained feasibility as the next design question.”'),
 S('State what each plot establishes, cite methods/tools and explain team/AI contributions. Keep decomposition, timing and turn tables as backup.')
], 'Presentation requirements supplied by the user; talk evidence [E12,E41–E43,E46].')
page('credit','Tools, reused ideas and contribution credit','Attribution • fill team roles accurately',[
 T([['Source / tool','What it contributes'],['Gazebo Harmonic / DART','Trajectory/contact simulation and finite held physics steps. Project bridge supplies the wheel interface/contact-surface behavior.'],['MuJoCo','Companion multibody model, mass/bias/Jacobian calculations, local model differentiation and visualization support.'],['SciPy / NumPy','Riccati solver, least-squares/numerical linear algebra and saved-array analysis.'],['Matplotlib / imageio / PDF tools','Figures, saved-state video/rendering and this study guide’s document generation.'],['De Luca and Mattone','Generalized-momentum residual/observer idea. The current discrete floating-base calculation is a project adaptation.'],['Tedrake / standard LQR theory','Local quadratic feedback concepts. LQR itself is not a project-invented algorithm.'],['Leo Rover / ERC Marsyard assets','Third-party terrain data/world assets used by the Mars simulation.'],['Supplied mechanism/CAD','Geometry/design provenance from the project’s supplied material; identify the actual designer/team in the slides.']],[158,317]),
 H('Project-specific implementation work'),
 P('The repository contains wheel topology/patch modeling, the simulation bridge, observer-to-support construction, contact-aware references, scheduled feedback integration, incoming-drive preparation and experiment/diagnostic tooling. Describe which parts each team member designed, modified, reviewed or validated from your own records; this document cannot infer personal authorship.'),
 H('AI assistance — truthful wording to adapt'),
 P('“We used Codex assistance for code implementation, debugging, experiment orchestration, analysis and drafting. Our presentation distinguishes established methods and library solvers from project-specific integration, and explains the evidence and design decisions we reviewed.”'),
 S('This describes assistance visible in the project conversation; adjust team-specific wording to match actual work. The goal is to understand and defend the implementation, not claim to have handwritten a library solver or derived a new control theorem.')
], 'Repository dependencies and source provenance [C1–C10]; primary references [T1–T7].')
page('references','Primary references to cite','Citations • concepts and third-party tools',[
 P('These references support general methods and software. Project-specific equations, parameters and numerical outcomes are traced separately to code and saved evidence on the following page. Retrieved/checked26 November2026.'),
 H('[T1] LQR and constrained-control background'),
 P('Russ Tedrake. <i>Underactuated Robotics</i>, chapter “Linear Quadratic Regulators.” MIT course notes. <link href="https://underactuated.mit.edu/lqr.html" color="#256F98">underactuated.mit.edu/lqr.html</link>.'),
 H('[T2] Momentum-residual contact background'),
 P('A. De Luca and R. Mattone. “Sensorless Robot Collision Detection and Hybrid Force/Motion Control.” ICRA2005, pp.999–1004. DOI:10.1109/ROBOT.2005.1570247. <link href="https://www.diag.uniroma1.it/~labrob/pub/papers/ICRA05_CollDetect.pdf" color="#256F98">Author-hosted paper</link>.'),
 H('[T3] Earlier generalized-momenta work'),
 P('A. De Luca and R. Mattone. “Actuator failure detection and isolation using generalized momenta.” ICRA2003, pp.634–639. DOI:10.1109/ROBOT.2003.1241665. <link href="https://doi.org/10.1109/ROBOT.2003.1241665" color="#256F98">DOI</link>. Bibliographic details verified against the author’s publication list; the PDF’s implementation claims are based on current code, not an assertion of exact paper equivalence.'),
 H('[T4–T5] Dynamics conventions and the Riccati solver'),
 P('<link href="https://mujoco.readthedocs.io/en/stable/computation/index.html" color="#256F98">MuJoCo computation documentation</link>: generalized coordinates, mass/bias/contact terms. <link href="https://docs.scipy.org/doc/scipy-1.15.3/reference/generated/scipy.linalg.solve_discrete_are.html" color="#256F98">SciPy1.15.3 solve_discrete_are</link>: discrete algebraic Riccati equation.'),
 H('[T6–T7] Simulation engine and terrain'),
 P('<link href="https://gazebosim.org/docs/harmonic/comparison/" color="#256F98">Gazebo Harmonic physics/features documentation</link>. <link href="https://github.com/LeoRover/leo_simulator-ros2" color="#256F98">LeoRover/leo_simulator-ros2</link>, leo_gz_worlds Marsyard2021 assets. See repository GAZEBO.md for the project’s DEM triangulation/collision adaptation and asset attribution.'),
 C('Do not cite this study guide as the original source of LQR or generalized momentum observers. Cite the method reference and the project implementation/evidence that supports each result.')
])
page('evidence','Where to find each claim in the repository','Implementation and saved evidence index',[
 T([['ID','Repository path / function'],['C1','config/model.yaml — topology assumptions, timing, friction and input bounds'],['C2','src/spherical_wheel/model.py; sim.py:Plant/error/apply_input'],['C3','src/spherical_wheel/control.py:47,66 — linearize/design_lqr'],['C4','src/spherical_wheel/recovery_control.py:172,272; offroad_control.py:303 — scheduled gain bank'],['C5','src/spherical_wheel/offroad_control.py:513,547,768 — references, command, terrain feedback'],['C6','src/spherical_wheel/experiments.py:29,79 — rolling allocation; rolling_allocation.py — experimental bounds'],['C7–C8','src/spherical_wheel/contact_sensing.py:201,227,379 — intrinsic normal, observer, locator'],['C9','src/spherical_wheel/synchronization.py:154 — active constant-velocity preview'],['C10','src/spherical_wheel/offroad_experiments.py:248; gazebo/plant.py:148,173; gazebo/bridge.py:82']],[55,420]),
 H('Experiment records under docs/iterations/'),
 P('<b>E12:</b>2026-11-25/configuration-study/README.md. <b>E41:</b>2026-11-25/figure8-event-audit/README.md. <b>E42:</b>2026-11-25/handoff-factorial/README.md. <b>E43:</b>2026-11-25/tilt-reference/README.md. <b>E46:</b>2026-11-26/support-routing-timing/README.md.'),
 H('New read-only audits under results/presentation_preparation_20261126/'),
 P('<b>A1:</b>controller_audit/controller_audit.md, report.json and saved_decomposition.json. <b>A2:</b>contact_audit/contact_observer_audit.md, contact_audit.json and historical/transition tables. <b>A3:</b>motion_audit/README.md, motion_audit.json, figures and qa.json.'),
 S('Raw prior NPZ inputs remain in their original ignored results families; audit manifests record their paths/hashes. This guide and compact evidence copies live under docs/iterations/2026-11-26/presentation-preparation/. Sources are uncommitted research work; snapshots identify the inspected versions. No new robot trial was run for this document.')
])
page('cost_appendix','The LQR error groups and weights','Technical backup • do not put this table on the main slides',[
 P('The nominal18-component error layout is below. The design normalizes each group, applies the listed Q weights and transforms the result back to physical units. The running controller then deliberately modifies selected error entries as explained earlier.'),
 T([['Error indices','Group','Scales','Q diagonal'],['0–2','Position x,y,z','.15,.15,.005','2,2,1'],['3–5','Attitude tangent x,y,z','.15,.15,.5','30,30,.01'],['6–8','Joint angle shaft,A,B','1,1,1','.001,.001,.001'],['9–11','Linear velocity','1,1,.1','2,2,.1'],['12–14','Angular velocity','1,1,1','4,4,.05'],['15–17','Joint angular velocity','10,10,10','.01,.01,.01']],[75,145,125,130]),
 P('The input normalization uses bounds[1.5,0.6,0.6,0.3,0.3]; R<sub>u</sub>=diag(0.3,0.3,0.3,1,1). Selected gains use the full18-coordinate nominal design followed by reachable-subspace reduction. An optional frozen_phase=True comparison branch omits selected yaw/material-angle coordinates; it is not active here. Runtime suppresses or changes selected errors separately, while logging all18 entries.'),
 H('Six definitions worth being able to say'),
 B('<b>Jacobian:</b> maps a small joint/base velocity into contact-point velocity; its transpose maps a contact force into generalized force.',
 '<b>Feedforward:</b> input/reference contribution computed from the desired motion or holding model, before observing the resulting tracking error.',
 '<b>Feedback:</b> correction based on current error relative to the chosen reference.',
 '<b>Observer:</b> uses measurements plus a model to estimate an unmeasured quantity.',
 '<b>Singular value:</b> measures gain along a particular independent input/output direction; a small value indicates weak instantaneous authority.',
 '<b>Constraint versus cost:</b> a constraint restricts the allowed set; a cost ranks choices inside the optimization’s allowed set.'),
 S('Study test: explain why a cap on a rate reference does not guarantee a torque bound, why torque clipping does not guarantee tracking, and why a small point error does not guarantee the correct support label.')
], '[C3,C5]; exact source citations and current numeric verification [A1].')
(HERE/'guide_content.json').write_text(json.dumps(dict(title='Active spherical wheel: current dynamics, control and estimation',pages=pages),indent=2,ensure_ascii=False)+'\n')
print('pages',len(pages))
