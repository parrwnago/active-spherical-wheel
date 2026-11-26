# I-35 final review and validation

2026-11-25 America/New_York. This stage is read-only review and archival;
no extra optimization, simulation or unit-test suite was executed.

An independent reviewer compared the protocol, merged report, gate CSV and
merge implementation. No material gate error was found. The late 0.8 s return
failure and both 1.2 s mechanical failures support rejection independently of
the early 40 mm cutoff or the first-input-change criterion. No Gazebo trial
qualifies. Preserve the distinction between predicted tipping and actual
Gazebo results, and between solved local QPs and nonlinear convergence.

The seed's 77–103 mm fine/coarse divergence, extreme Hessian scales and fixed
refinement budget limit conclusions about the value of longer horizons.
Negative computed Hessian minima at those scales do not establish physical
nonconvexity. Intermediate accepted motion trajectories were not saved; that
limitation remains explicit and no solves were repeated to fill it.

Final lightweight checks passed:

- `.venv/bin/python -m py_compile scripts/diagnose-mpc-horizon.py`
- `.venv/bin/python scripts/diagnose-mpc-horizon.py --help`
- `git diff --check`

The compiler/help checks are not new unit tests. Worker provenance and sealed
artifact/prefix/replay records are verified again by `archive_evidence.py`.
Archive payload hashes and ZIP CRC are verified by `package_presentation.py`.
Earlier I-32/I-33 and I-34 packs must retain the hashes in the protocol.
