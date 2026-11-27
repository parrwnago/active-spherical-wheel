# I-66 data boundary: existing contact truth and load records

This is a read-only schema/provenance/source audit. No new residual arithmetic, calibration, scoring, kinematic reconstruction, contact query or physics run was performed. `schema_inventory.json` records selected NPZ headers, paths, source hashes and per-case provenance for m100_B, p030_B and p090_B. Headers were inspected directly without loading data arrays for numerical analysis.

## What is available

The raw paths for each case are `results/balance_priority_20261126/campaign/runs/CASE/precision.npz` and `rollout.npz`; all have8,801 control observations for the44s I-58 native Gazebo/DART run.

| Packet/field | Stored shape/type | Meaning |
|---|---|---|
| precision: time_s,truth_time_s | N,float64 | Matching5ms macro/control clock; no per-contact timestamp |
| precision: exact_qpos,exact_qvel | N×10,N×9,float64 | Endpoint measured simulator state. Position/world linear velocity; quaternion; body-frame free angular velocity and scalar joint rates |
| precision: exact_observer_generalized,exact_observer_interval_mean | N×9,float64 | Filtered and preceding-control-interval model-inferred external loads; not solved contact truth |
| precision: exact_observer_wrench | N×6,float64 | Filtered inferred world force and centre moment |
| precision: truth_patch_loads_N | N×4,float64 | Sum of compressive normal pressure for shell_A,shell_B,belt_A,belt_B |
| precision: truth_patch_points_m | N×4×3,float64 | Per-family NORMAL-LOAD-WEIGHTED contact-position centroid; NaN with no load |
| precision: truth_patch_contact_counts | N×4,int64 | Number of matching contact records whose individual normal force exceeds0.05N |
| precision: truth_equivalent_point_m | N×3,float64 | Normal-load-weighted centroid over wheel-support records, not generally a point reproducing the net wrench |
| rollout: contact_force_N | N×3,float32 | Sum of world force vectors, including tangential components, on the four wheel-support families |
| rollout: contact_moment_Nm | N×3,float32 | Sum of (point−current wheel centre) cross world force over those records |
| rollout: normal_force_N,support_loads_N | N,N×4,float32 | Compressive normal-load sum and family sums |
| rollout: tangential_force_N | N,float32 | Norm of NET tangential component relative to pressure-weighted normal; not per-contact/per-family tangential vectors |
| rollout: fork_load_N | N,float32 | Compressive load on unsupported/nonwheel records; these are excluded from the saved wheel resultant force/moment |
| rollout: contact_point_m | N×3,float32 | Same normal-load centroid as the float64 truth-equivalent field |

The saved truth positions include all nonnegative matching normal loads; the count includes only loads strictly greater than0.05N. One resolved record therefore does not prove one physical contact. Centroids may lie inside the sphere or outside the assigned material patch and need not reproduce net moment when tangential forces vary across positions. Preserve them raw and report physical-material validity; a radial projection is a model proxy, not saved truth.

The recorded aggregate moment can contain the moment of distributed point forces. No separate contact couple/torque vector is exported by the bridge or stored in these packets. It is not permissible to interpret the saved aggregate moment as a per-body or per-contact torque decomposition. Casting float32 solved force/moment to float64 cannot restore the original precision.

## Timing and producer proof

Use the ARCHIVED native case source, not a later current implementation. Under `results/balance_priority_20261126/campaign/runs/m100_B/source_snapshot/`:

- `config/model.yaml` and `src/spherical_wheel/sim.py`: physics step0.001s, control interval0.005s, five substeps.
- `src/spherical_wheel/gazebo/plant.py:173-183`: apply one held command, run all five substeps, then pull state and contacts once. `_pull:148-162` copies only the bridge's latest contact array.
- `gazebo/plugin/SphericalWheelBridge.cc:264-312`: every PostUpdate clears the shared contact list and replaces it with that completed physics step's records. The exported16-column contact row at548-570 contains tag,position,normal,force,depth,friction,admission and three zero placeholders. Only the force part of the message wrench is read.
- `src/spherical_wheel/offroad_experiments.py:248-305`: measure and append once per5ms observation, then advance. `save_run:401-410` deliberately converts float64 rollout arrays to float32.
- `scripts/run-lqr-contact-forecast.py:142-162`: pressure-weighted per-family centroids, >0.05N resolved counts and equivalent centroid definitions.
- `src/spherical_wheel/contact_sensing.py:153-198`: force/moment sums, wheel-support filtering and normal-weighted aggregate point/normal.
- `results/balance_priority_20261126/campaign_tools/run_case.py:214-218,312-327`: capture truth once through measure_contact, assert macro timestamps match, write precision. It never instantiates or attaches IntervalTruthSampler.

Thus the truth arrays are the last reported completed1ms solver-step aggregate at each5ms observation; they are NOT five-step averages. The observer interval_mean instead estimates load over the preceding5ms via the dynamics model. Snapshot timestamps matching does not make these temporal quantities identical. Full contact positions, normals and forces are consumed by the logger but are not persisted individually.

An IntervalTruthSampler class exists in the archived source, but presence in a snapshot is not execution. Other old campaigns have genuine substep packets: `results/causal_replay_20261126/campaign/native_substeps/substeps.npz` belongs to the older guarded m100_R path and a limited32s-era window; `results/support_routing_timing_20261126/.../truth_substeps.npz` belongs to flat/Mars compact out-and-back experiments. Neither is the missing history of the three native B runs. Historical I-56/I-58 shared endpoint precision equality does not add absent substep fields.

## Derived packets do not add missing truth

`results/velocity_contact_rank_20261127/campaign/features_stage01/CASE/measurement_context.npz` preserves exact q/qdot, raw/filtered inferred loads and the observer-conditioned intrinsic normal, plus known prior input. Its evaluation packet copies normal-load truth and labels; it does not contain contact-force routing.

`results/contact_load_consistency_20261127/analysis/stage01/CASE_evaluation.npz` preserves raw equivalent/dominant-family centroids and truth labels. The dominant-family point is selected from truth_patch_points_m, not an individual contact. Its radial-R proxy fields are explicitly projections. Symmetric stable_truth_mask is evaluation-only.

The I-61 feature packet has `map__centre_m`, `map__joint_moment_map[N,4,3,3]` and `map__joint_at_centre[N,4,3]`. The centre term is already multiplied by the FILTERED observed force, while the moment map is a kinematic angular map. Do not silently reuse that force-conditioned centre term for the raw interval force. Existing I-64 caches hold full H for frozen F/Nalt/N_equivalent points, but not arbitrary sampled-centroid points. New private kinematics/Jacobian caching at the saved q is possible without querying contacts or stepping physics, after source/input sealing.

## Smallest defensible next diagnostic

Compare fixed candidate F/Nalt and sampled dominant-family RAW centroid using the SAME exact endpoint q and SAME observed raw interval force. Call this sampled-centroid geometry sensitivity or reference, not an exact5ms oracle. To distinguish point geometry from assigned-body routing, hold one fixed while varying the other explicitly. A centroid mapped to a nonmatching body is a virtual assigned-body diagnostic and must carry its material-validity qualification.

The saved float32 resultant force/moment permits a separately labeled endpoint-force or point-wrench consistency sensitivity. It cannot isolate observer error from the1ms-versus5ms mismatch. Normal-load shares may support an assumed split model, but they are not per-family vector-force truth and do not recover tangential correlations or body routing.

Exact previous5ms generalized contact load cannot be reconstructed from these packets. The missing information includes all five substep configurations and contact assignments, individual force vectors/points/normals and their correlations, plus any unexported contact torque. Summing endpoint centroids or interpolating them would introduce assumptions, not recover truth. A definitive interval-force/body-routing decomposition would need a separately authorized capture with substep records; it is not part of this audit.
