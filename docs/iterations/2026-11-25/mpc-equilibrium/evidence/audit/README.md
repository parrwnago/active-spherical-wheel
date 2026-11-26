# I-31 frozen estimated-plane equilibrium audit

Completed five declared private-model anchors: world Y at 1.4, 2, 3, and 5 s; world X at 2 s. Zero of five candidate rest targets passed the held-input gates. Goal XY and the already calibrated estimated plane remained fixed during each local solve. Optimizer success alone was never accepted as an equilibrium.

The best-found selected candidates move 0.986–1.513 mm and rotate 0.426–0.608 degrees in 0.2 s, against gates of 0.2 mm and 0.1 degrees. All also exceed body-speed, angular-rate, and internal-rate gates. Wheel support and bounded input gates pass. All nine initial accelerations, inverse-force diagnostics, input margins, contact identities, load ranges, slip diagnostics, both local solve attempts, and sampled private holds are preserved in report.json. These bounded local search failures do not prove physical goal infeasibility.

No terminal controller was derived or adopted, and no closed-loop Gazebo performance run was launched by this audit. Parent-owned matched-reset fixed-input Gazebo diagnostics are a separate experiment.

Files: report.json (full evidence), summary.json / hold_summary.csv (compact values), hold_audit.png/pdf (gate plot), declared_source/manifest.json (exact source before the completed five-anchor audit). The two failed audit setup stages and their source snapshots remain alongside the successful audit. Final validation-only module changes and 17 new tests are captured in ../tests/final_validated_equilibrium_source; do not label them as the exact earlier five-anchor source.

Validation: the final selected integration suite passed 106 tests in 50.27 s. Its exact pre-launch source is ../tests/pre_final_integration_source. This is not the whole repository suite, and overlaps earlier selected checks. Parent subsequently added an optional OSQP test guard and reran those two tests; the later file is separately labeled.

Reproduction: the audit script refuses overwriting completed declared_source. To rerun a fresh stage, preserve this folder, change OUTPUT in a copied script, and run build/gazebo-env/bin/python scripts/audit-mpc-equilibrium.py with the matching repository source and original trajectory inputs. Full input hashes are recorded per anchor. The plot can be regenerated without physics using .venv/bin/python results/mpc_equilibrium_20261125/audit/plot_audit.py.
