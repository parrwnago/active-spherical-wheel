# Pre-outcome confidence denominator clarification

This addendum was written before numeric data or results were read. It preserves
the original blind review instead of silently editing its initial suggestions.

The suggested 50% confidence-coverage gate should use the fixed population with
informative measured motion and at least two geometrically valid **actuator
families**, rather than every moving state. Otherwise correct geometry-singleton
states would make velocity-disambiguation coverage fail for an irrelevant
reason. Coverage over all truth-eligible single-support states must also be
reported separately. The parent protocol supplies the final adopted gate.

Geometric face and rim kinds that command the same shell collapse to one
actuator family before ranking/margin/confidence. There are four actuator
families, even if there are six geometric kinds and multiple normal samples.
The runner-up must be a different actuator family. Combine a candidate's score
components at that same point/normal, then minimize within each family; do not
construct an impossible winner from the best component of different candidates.

Measured motion alone does not guarantee identity observability. Distinct
candidate families can predict identical rolling velocities while the robot
is moving. A fixed residual-vector contrast requirement makes that ambiguity
explicit. It is still only a consistency gate and cannot establish correctness
under slip.

The recorded original support locator is a separate baseline. A new geometry
ranker with a normal uncertainty grid is not automatically the same algorithm,
even if the two sometimes choose the same family. Preserve their distinct
labels and denominators.
